/*
 * u_uac1.h -- interface to USB gadget "ALSA AUDIO" utilities
 *
 * Copyright (C) 2008 Bryan Wu <cooloney@kernel.org>
 * Copyright (C) 2008 Analog Devices, Inc
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __U_AUDIO_H
#define __U_AUDIO_H

#include <linux/device.h>
#include <linux/err.h>
#include <linux/usb/audio.h>
#include <linux/usb/composite.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#include "gadget_chips.h"

#define QUECTEL_UAC_FEATURE //add yang.yang 2018-01-05 uac

#define FILE_PCM_PLAYBACK	"/dev/snd/pcmC0D5p"
#define FILE_PCM_CAPTURE	"/dev/snd/pcmC0D6c"
#define FILE_CONTROL		"/dev/snd/controlC0"

//#define FILE_PCM_PLAYBACK	"/dev/snd/pcmC0D0p"
//#define FILE_PCM_CAPTURE	"/dev/snd/pcmC0D1c"
//#define FILE_CONTROL		"/dev/snd/controlC0"


#define UAC1_IN_EP_MAX_PACKET_SIZE	32
#define UAC1_OUT_EP_MAX_PACKET_SIZE	32
#define UAC1_OUT_REQ_COUNT		48
#define UAC1_IN_REQ_COUNT		 4
#define UAC1_AUDIO_PLAYBACK_BUF_SIZE  256 //480 //320// 256	/* Matches with Audio driver */
#define UAC1_AUDIO_CAPTURE_BUF_SIZE   256 //480 //320 //256	/* Matches with Audio driver */
#define UAC1_SAMPLE_RATE	     8000
/*
 * This represents the USB side of an audio card device, managed by a USB
 * function which provides control and stream interfaces.
 */

struct gaudio_snd_dev {
	struct gaudio			*card;
	struct file			*filp;
	struct snd_pcm_substream	*substream;
	int				access;
	int				format;
	int				channels;
	int				rate;
};

struct gaudio {
	struct usb_function		func;
	struct usb_gadget		*gadget;

	/* ALSA sound device interfaces */
	struct gaudio_snd_dev		control;
	struct gaudio_snd_dev		playback;
	struct gaudio_snd_dev		capture;

	bool				audio_reinit_capture;
	bool				audio_reinit_playback;
#ifdef QUECTEL_UAC_FEATURE
	int					usb_snd_opened;
	int					usb_snd_setuped;
#endif

	/* TODO */
};

struct f_uac1_opts {
	struct usb_function_instance	func_inst;
	int				req_playback_buf_size;
	int				req_capture_buf_size;
	int				req_playback_count;
	int				req_capture_count;
	int				audio_playback_buf_size;
	int				audio_capture_buf_size;
	int				audio_playback_realtime;
	int				sample_rate;
	char				*fn_play;
	char				*fn_cap;
	char				*fn_cntl;
	unsigned			bound:1;
	unsigned			fn_play_alloc:1;
	unsigned			fn_cap_alloc:1;
	unsigned			fn_cntl_alloc:1;
	struct gaudio			*card;
	struct mutex			lock;
	int				refcnt;
};

int gaudio_setup(struct gaudio *card);
void gaudio_cleanup(struct gaudio *the_card);

void u_audio_clear(struct gaudio *card);
size_t u_audio_playback(struct gaudio *card, void *buf, size_t count);
size_t u_audio_capture(struct gaudio *card, void *buf, size_t count);
int u_audio_get_playback_channels(struct gaudio *card);
int u_audio_get_playback_rate(struct gaudio *card);
int u_audio_get_capture_channels(struct gaudio *card);
int u_audio_get_capture_rate(struct gaudio *card);

#endif /* __U_AUDIO_H */
