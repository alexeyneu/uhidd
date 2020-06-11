  /*-
 * Copyright (c) 2009 Kai Wang
 * Copyright (c) 2020 Alex Neudatchin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in this position and unchanged.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD: trunk/uvhid/uvhid.c 36 2009-07-29 02:59:57Z kaiw27 $");

#include <sys/param.h>
#include <sys/conf.h>
#include <sys/proc.h>
#include <sys/condvar.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/poll.h>
#include <sys/thread.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>
#include <sys/uio.h>
#include <sys/queue.h>
#include <bus/u4b/usb_ioctl.h>
#include <sys/device.h>
#include <sys/devfs.h>
#include "uvhid_var.h"
#include <sys/lock.h>

#define	UVHID_NAME	"uvhid"
#define	UVHIDCTL_NAME	"uvhidctl"
#define	UVHID_QUEUE_SIZE	10240
#define	UVHID_MAX_REPORT_SIZE	255
#define UVHID_MAX_REPORT_DESC_SIZE	10240

MALLOC_DECLARE(M_UVHID);
MALLOC_DEFINE(M_UVHID, UVHID_NAME, "Virtual USB HID device");

#define UVHID_LOCK_GLOBAL()	mtx_lock(&uvhidmtx);
#define UVHID_UNLOCK_GLOBAL()	mtx_unlock(&uvhidmtx);
//#define UVHID_LO K(s)		mtx_lock(&(s)->us_mtx)
//#define UVHID_UNL OCK(s)		mtx_unlock(&(s)->us_mtx)
#define UVHID_LOCK_ASSERT(s, w)	mtx_assert(&(s)->us_mtx, w)
#define	UVHID_SLEEP(s, f, d, t) \
	msleep(&(s)->f, &(s)->us_mtx, PCATCH | (PZERO + 1), d, t)

DEVFS_DEFINE_CLONE_BITMAP(uvhidctl);

struct rqueue {
	int		cc;
	unsigned char	q[UVHID_QUEUE_SIZE];
	unsigned char	*head;
	unsigned char	*tail;
};
struct lock lockn;

struct uvhid_softc {
	struct rqueue	us_rq;
	struct rqueue	us_wq;
	struct kqinfo	us_rsel;
	struct kqinfo	us_wsel;
	struct lock	us_lock;
	struct cv	us_cv;
	
	int		us_hcflags;
	int		us_hflags;

#define	OPEN	(1 << 0)	/* device is open */
#define	READ	(1 << 1)	/* read pending */
#define	WRITE	(1 << 2)	/* write pending */

	unsigned char	us_rdesc[UVHID_MAX_REPORT_DESC_SIZE];
	int		us_rsz;
	int		us_rid;

	STAILQ_ENTRY(uvhid_softc) us_next;
};

/* Global mutex to protect the softc list. */
static STAILQ_HEAD(, uvhid_softc) hidhead = STAILQ_HEAD_INITIALIZER(hidhead);


static int	gen_read(struct uvhid_softc *sc, int *scflag,
		    struct rqueue *rq, struct uio *uio, int flag);
static int	gen_write(struct uvhid_softc *sc, int *scflag,
		    struct rqueue *rq, struct kqinfo *sel, struct uio *uio,
		    int flag);
static void	rq_reset(struct rqueue *rq);
static void	rq_dequeue(struct rqueue *rq, char *dst, int *size);
static void	rq_enqueue(struct rqueue *rq, char *src, int size);

static d_clone_t	hidctl_clone;

static d_open_t		hidctl_open;
static d_close_t	hidctl_close;
static d_read_t		hidctl_read;
static d_write_t	hidctl_write;
static d_ioctl_t	hidctl_ioctl;
static d_kqfilter_t hidctl_kqfilter;


static d_open_t		hid_open;
static d_close_t	hid_close;
static d_read_t		hid_read;
static d_write_t	hid_write;
static d_ioctl_t	hid_ioctl;
static d_kqfilter_t hid_kqfilter;

static struct dev_ops hidctl_cdevsw = {
	.head = {UVHIDCTL_NAME},
	.d_open	   = hidctl_open,
	.d_close   = hidctl_close,
	.d_read	   = hidctl_read,
	.d_write   = hidctl_write,
	.d_ioctl   = hidctl_ioctl,
	.d_kqfilter = hidctl_kqfilter,
};

static struct dev_ops hid_cdevsw = {
	.head = {UVHID_NAME},
	.d_open	   = hid_open,
	.d_close   = hid_close,
	.d_read	   = hid_read,
	.d_write   = hid_write,
	.d_ioctl   = hid_ioctl,
	.d_kqfilter = hid_kqfilter,
};


static int
hidctl_clone(struct dev_clone_args *t)
{
	struct cdev *hid_dev;
	int unit;
	
	unit = devfs_clone_bitmap_get(&DEVFS_CLONE_BITMAP(uvhidctl), 0);

	t->a_dev = make_only_dev(&hidctl_cdevsw, unit, UID_ROOT, GID_WHEEL, 0600,
				  UVHIDCTL_NAME "%d", unit);
	reference_dev(t->a_dev);
	
	hid_dev = make_dev(&hid_cdevsw, unit, UID_ROOT, GID_WHEEL, 0700,
				  UVHID_NAME "%d", unit);
	reference_dev(hid_dev);

	struct uvhid_softc *sc;

	sc = kmalloc(sizeof(*sc), M_UVHID, M_WAITOK|M_ZERO);
	lockinit(&sc->us_lock, "uvhidctl lock", 0, LK_CANRECURSE);
	cv_init(&sc->us_cv, "uvhidcv");
	t->a_dev->si_drv1 = sc;
	hid_dev->si_drv1 = sc;
	lockmgr(&lockn, LK_EXCLUSIVE);
	STAILQ_INSERT_TAIL(&hidhead, sc, us_next);
	lockmgr(&lockn, LK_RELEASE);

	return 0;
}

static void
hidctl_destroy(struct uvhid_softc *sc)
{

	if (((sc->us_hcflags & OPEN) != 0) || ((sc->us_hflags & OPEN) != 0))
	{
		lockmgr(&sc->us_lock, LK_EXCLUSIVE|LK_CANRECURSE);
		cv_wait(&sc->us_cv, &sc->us_lock);
		lockmgr(&sc->us_lock, LK_RELEASE);
	}
	cv_destroy(&sc->us_cv);
	lockuninit(&sc->us_lock);	
	kfree(sc, M_UVHID);
}

static int
hidctl_open(struct dev_open_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;

	lwkt_getpooltoken(sc);
	if (sc->us_hcflags & OPEN) {
		lwkt_relpooltoken(sc);
		return (EBUSY);
	}
	sc->us_hcflags |= OPEN;
	rq_reset(&sc->us_rq);
	rq_reset(&sc->us_wq);
	lwkt_relpooltoken(sc);

	return (0);
}

static int
hidctl_close(struct dev_close_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	struct klist *klist = &sc->us_rsel.ki_note;
	lwkt_getpooltoken(sc);
	sc->us_hcflags &= ~OPEN;
	rq_reset(&sc->us_rq);
	rq_reset(&sc->us_wq);
	KNOTE(klist, 0);
	if ((sc->us_hflags & OPEN) == 0)
		cv_broadcast(&sc->us_cv);
	lwkt_relpooltoken(sc);

	return (0);
}

static int
hidctl_read(struct dev_read_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;

	return (gen_read(sc, &sc->us_hcflags, &sc->us_wq, t->a_uio, t->a_ioflag));
}

static int
hidctl_write(struct dev_write_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	return (gen_write(sc, &sc->us_hcflags, &sc->us_rq, &sc->us_rsel, t->a_uio,
	    t->a_ioflag));
}

static int
hidctl_ioctl(struct dev_ioctl_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	struct usb_gen_descriptor *ugd;
	unsigned char *rdesc;
	int err;

	err = 0;

	switch ( t->a_cmd ) {
	case USB_SET_REPORT_DESC:

		ugd = (struct usb_gen_descriptor *)t->a_data;
		if (ugd->ugd_actlen == 0)
			break;
		if (ugd->ugd_actlen > UVHID_MAX_REPORT_DESC_SIZE) {
			err = ENXIO;
			break;
		}
		rdesc = kmalloc(UVHID_MAX_REPORT_DESC_SIZE, M_UVHID, M_WAITOK);
		err = copyin(ugd->ugd_data, rdesc, ugd->ugd_actlen);
		if (err) {
			kfree(rdesc, M_UVHID);
			break;
		}
		lwkt_getpooltoken(sc);
		bcopy(rdesc, sc->us_rdesc, ugd->ugd_actlen);
		sc->us_rsz = ugd->ugd_actlen;
		lwkt_relpooltoken(sc);
		kfree(rdesc, M_UVHID);
		break;

	case USB_SET_REPORT_ID:
		lwkt_getpooltoken(sc);
		sc->us_rid = *(int *)t->a_data;
		lwkt_relpooltoken(sc);
		break;

	default:
		err = ENOIOCTL;
		break;
	}

	return (err);
}



static void filt_hidctldetach(struct knote *kn);
static int filt_hidctlread(struct knote *kn, long hint);
static int filt_hidctlwrite(struct knote *kn, long hint);

static struct filterops hidctlread_filtops =
	{ FILTEROP_ISFD ,
	  NULL, filt_hidctldetach, filt_hidctlread };

static struct filterops hidctlwrite_filtops =
	{ FILTEROP_ISFD ,
	  NULL, filt_hidctldetach, filt_hidctlwrite };


static int
hidctl_kqfilter(struct dev_kqfilter_args *t)
{
	struct knote *kn = t->a_kn;
	cdev_t dev = t->a_head.a_dev;
	struct uvhid_softc *sc = dev->si_drv1;
		
	t->a_result = 0;
	struct klist *klist = &sc->us_wsel.ki_note;

	switch (kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &hidctlread_filtops;
		break;
	case EVFILT_WRITE:
		kn->kn_fop = &hidctlwrite_filtops;
		break;
	default:
		t->a_result = EOPNOTSUPP;
		return (0);
	}
	kn->kn_hook = (caddr_t)t->a_head.a_dev;	
	knote_insert(klist, kn);
	return (0);

}

static void
filt_hidctldetach(struct knote *kn) 
{
	cdev_t dev = (cdev_t)kn->kn_hook;
	struct uvhid_softc *sc = dev->si_drv1;
	struct klist *klist = &sc->us_wsel.ki_note;

	knote_remove(klist, kn);

}

static int
filt_hidctlread(struct knote *kn, long hint)
{
	cdev_t dev = (cdev_t)kn->kn_hook;
	struct uvhid_softc *sc = dev->si_drv1;

	if(kn->kn_sfflags & NOTE_OLDAPI)
	{
		lwkt_getpooltoken(sc);
		if (sc->us_wq.cc > 0)
		{
			lwkt_relpooltoken(sc);
			return 1;
		}
		else;
//			....
		lwkt_relpooltoken(sc);
	}
	return 0;
}

static int
filt_hidctlwrite(struct knote *kn, long hint)
{
	if(kn->kn_sfflags & NOTE_OLDAPI)
		return 1;
	return 0;
}

static int
hid_open(struct dev_open_args* t)
{

	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	
	lwkt_getpooltoken(sc);
	if (sc->us_hflags & OPEN) {
		lwkt_relpooltoken(sc);
		return (EBUSY);
	}
	sc->us_hflags |= OPEN;
	lwkt_relpooltoken(sc);

	return (0);
}

static int
hid_close(struct dev_close_args* t)
{
	
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	struct klist *klist; 	
	klist = &sc->us_wsel.ki_note;

	lwkt_getpooltoken(sc);
	sc->us_hflags &= ~OPEN;
	KNOTE(klist, 0);
	if ((sc->us_hcflags & OPEN) == 0)
		cv_broadcast(&sc->us_cv);
	lwkt_relpooltoken(sc);

	return (0);
}

static int
hid_read(struct dev_read_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;

	return (gen_read(sc, &sc->us_hflags, &sc->us_rq, t->a_uio, t->a_ioflag));
}

static int
hid_write(struct dev_write_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;

	return (gen_write(sc, &sc->us_hflags, &sc->us_wq, &sc->us_wsel, t->a_uio,
	    t->a_ioflag));
}

static int
hid_ioctl(struct dev_ioctl_args* t)
{
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	struct usb_gen_descriptor *ugd;
	unsigned char *rdesc;
	int err;

	err = 0;

	switch ( t->a_cmd ) {
	case USB_GET_REPORT_DESC:
		rdesc = kmalloc(UVHID_MAX_REPORT_DESC_SIZE, M_UVHID, M_WAITOK);
		lwkt_getpooltoken(sc);
		ugd = (struct usb_gen_descriptor *)t->a_data;
		ugd->ugd_actlen = min(sc->us_rsz, ugd->ugd_maxlen);
		if (ugd->ugd_data == NULL || ugd->ugd_actlen == 0) {
			kfree(rdesc, M_UVHID);
			lwkt_relpooltoken(sc);
			break;
		}
		bcopy(sc->us_rdesc, rdesc, ugd->ugd_actlen);
		lwkt_relpooltoken(sc);
		err = copyout(rdesc, ugd->ugd_data, ugd->ugd_actlen);
		kfree(rdesc, M_UVHID);
		break;

	case USB_SET_IMMED:
	case USB_GET_REPORT:
	case USB_SET_REPORT:
		err = ENODEV;	/* not supported. */
		break;
			
	case USB_GET_REPORT_ID:
		lwkt_getpooltoken(sc);
		*(int *)t->a_data = sc->us_rid;
		lwkt_relpooltoken(sc);
		break;

	default:
		err = ENOIOCTL;
		break;
	}

	return (err);
}



static void filt_hiddetach(struct knote *kn);
static int filt_hidread(struct knote *kn, long hint);
static int filt_hidwrite(struct knote *kn, long hint);

static struct filterops hidread_filtops =
	{ FILTEROP_ISFD ,
	  NULL, filt_hiddetach, filt_hidread };

static struct filterops hidwrite_filtops =
	{ FILTEROP_ISFD ,
	  NULL, filt_hiddetach, filt_hidwrite };


static int
hid_kqfilter(struct dev_kqfilter_args *t)
{
	struct knote *kn = t->a_kn;
	t->a_result = 0;
	struct uvhid_softc *sc = t->a_head.a_dev->si_drv1;
	struct klist *klist = &sc->us_rsel.ki_note;		

	switch (kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &hidread_filtops;
		break;
	case EVFILT_WRITE:
		kn->kn_fop = &hidwrite_filtops;
		break;
	default:
		t->a_result = EOPNOTSUPP;
		return (0);
	}

	kn->kn_hook = (caddr_t)t->a_head.a_dev;
	knote_insert(klist, kn);
	return (0);

}

static void
filt_hiddetach(struct knote *kn) 
{
	cdev_t dev = (cdev_t)kn->kn_hook;
	struct uvhid_softc *sc = dev->si_drv1;
	struct klist *klist = &sc->us_rsel.ki_note;	
	knote_remove(klist, kn);
}

static int
filt_hidread(struct knote *kn, long hint)
{
	cdev_t dev = (cdev_t)kn->kn_hook;
	struct uvhid_softc *sc = dev->si_drv1;

	if(kn->kn_sfflags & NOTE_OLDAPI)
	{
		lwkt_getpooltoken(sc);
			if (sc->us_rq.cc > 0)
			{
				lwkt_relpooltoken(sc);
				return 1;
			}
			else;
//				....
		lwkt_relpooltoken(sc);
	}
	return 0;
}

static int
filt_hidwrite(struct knote *kn, long hint)
{
	if(kn->kn_sfflags & NOTE_OLDAPI)
		return 1;
	return 0;
}


static int
gen_read(struct uvhid_softc *sc, int *scflag, struct rqueue *rq,
    struct uio *uio, int flag)
{
	unsigned char buf[UVHID_MAX_REPORT_SIZE];
	int amnt,len, err = 0;

	lwkt_getpooltoken(sc);
	if (*scflag & READ) {
		lwkt_relpooltoken(sc);
		return (EALREADY);
	}
	*scflag |= READ;

	len = 0;

read_again:
	if (rq->cc > 0) {
		rq_dequeue(rq, buf, &len);
		lwkt_relpooltoken(sc);
		amnt = min(uio->uio_resid, len);
		err = uiomove(buf, amnt, uio);
		lwkt_getpooltoken(sc);
		} 
	else {
		if (flag & O_NONBLOCK) {
			err = EWOULDBLOCK;
			goto read_done;
		}

		lockmgr(&sc->us_lock, LK_EXCLUSIVE|LK_CANRECURSE);
		lksleep(&rq->cc,&sc->us_lock , PCATCH, "uvhidr", 0);
		lockmgr(&sc->us_lock, LK_RELEASE);

		if (err != 0)
			goto read_done;
		goto read_again;
	}

read_done:
	*scflag &= ~READ;
	lwkt_relpooltoken(sc);

	return (err);
}

static int
gen_write(struct uvhid_softc *sc, int *scflag, struct rqueue *rq,
    struct kqinfo *sel, struct uio *uio, int flag)
{
	unsigned char buf[UVHID_MAX_REPORT_SIZE];
	int err, len;
	struct klist *klist = &sel->ki_note;
	lwkt_getpooltoken(sc);

	if (*scflag & WRITE) {
		lwkt_relpooltoken(sc);
		return (EALREADY);
	}
	*scflag |= WRITE;

	lwkt_relpooltoken(sc);
	len = uio->uio_resid;
	err = uiomove(buf, len, uio);
	lwkt_getpooltoken(sc);
	if (err != 0)
		goto write_done;
	rq_enqueue(rq, buf, len);
	KNOTE(klist, 0);
	wakeup(&rq->cc);

write_done:
	*scflag &= ~WRITE;
	lwkt_relpooltoken(sc);

	return (err);
}


static void
rq_reset(struct rqueue *rq)
{

	rq->cc = 0;
	rq->head = rq->tail = rq->q;
}

static void
rq_dequeue(struct rqueue *rq, char *dst, int *size)
{
	int len, part1, part2;

	if (rq->cc == 0) {
		if (size)
			*size = 0;
		return;
	}

	len = (unsigned char) *rq->head++;
	rq->cc--;
	if (len > rq->cc)
		len = rq->cc;
	if (len == 0) {
		if (size)
			*size = 0;
		return;
	}

	part1 = UVHID_QUEUE_SIZE - (rq->head - rq->q);
	if (part1 > len)
		part1 = len;
	part2 = len - part1;

	if (part1 > 0) {
		if (dst) {
			memcpy(dst, rq->head, part1);
			dst += part1;
		}
		rq->head += part1;
		rq->cc -= part1;
	}

	if (rq->head == rq->q + UVHID_QUEUE_SIZE)
		rq->head = rq->q;

	if (part2 > 0) {
		if (dst)
			memcpy(dst, rq->head, part2);
		rq->head += part2;
		rq->cc -= part2;
	}

	if (size)
		*size = len;
}

static void
rq_enqueue(struct rqueue *rq, char *src, int size)
{
	int len, part1, part2;

	if (size > UVHID_MAX_REPORT_SIZE || size <= 0)
		return;

	/*
	 * Discard the oldest reports if not enough room left.
	 */
	len = size + 1;
	while (len > UVHID_QUEUE_SIZE - rq->cc)
		rq_dequeue(rq, NULL, NULL);

	part1 = UVHID_QUEUE_SIZE - (rq->tail - rq->q);
	if (part1 > len)
		part1 = len;
	part2 = len - part1;

	if (part1 > 0) {
		*rq->tail++ = (unsigned char) size;
		rq->cc++;
		part1--;

		memcpy(rq->tail, src, part1);
		src += part1;
		rq->tail += part1;
		rq->cc += part1;
	}

	if (rq->tail == rq->q + UVHID_QUEUE_SIZE)
		rq->tail = rq->q;

	if (part2 > 0) {
		if (part1 == 0) {
			*rq->tail++ = (unsigned char) size;
			rq->cc++;
			part2--;
		}

		memcpy(rq->tail, src, part2);
		rq->tail += part2;
		rq->cc += part2;
	}
}


static int
uvhid_modevent(module_t mod, int type, void *data)
{
	struct uvhid_softc *sc;
	static cdev_t devhc = NULL;

	switch (type) {
	case MOD_LOAD:
		lockinit(&lockn, "uvhidctl lock", 0, 0);
		devhc = make_autoclone_dev(&hidctl_cdevsw, &DEVFS_CLONE_BITMAP(uvhidctl),
					hidctl_clone , UID_ROOT, GID_WHEEL,0600, UVHIDCTL_NAME);
		break;

	case MOD_UNLOAD:
	case MOD_SHUTDOWN:
		lockmgr(&lockn, LK_EXCLUSIVE);
		while ((sc = STAILQ_FIRST(&hidhead)) != NULL) {
			STAILQ_REMOVE(&hidhead, sc, uvhid_softc, us_next);
			lockmgr(&lockn, LK_RELEASE);
			hidctl_destroy(sc);
			lockmgr(&lockn, LK_EXCLUSIVE);
		}
		lockmgr(&lockn, LK_RELEASE);
		dev_ops_remove_all(&hidctl_cdevsw);
		dev_ops_remove_all(&hid_cdevsw);
		destroy_autoclone_dev(devhc, &DEVFS_CLONE_BITMAP(uvhidctl));
		lockuninit(&lockn);
		break;

	default:
		return (EOPNOTSUPP);
	}

	return (0);
}

DEV_MODULE(uvhid, uvhid_modevent, NULL);
