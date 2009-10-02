/*
 * Copyright (c) 2008 Maxime COSTE <frrrwww@gmail.com>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/**
 * @file audio_format_musepack.c
 * @brief musepack decompression routines.
 */

#include "stdinc.h"

#include <mpcdec/mpcdec.h>

#include "audio_file.h"
#include "audio_format.h"
#include "audio_output.h"

#define MPC_BUFFER_SIZE 8912*2

/**
 * @brief Private Musepack data stored in the audio file structure.
 */
struct musepack_drv_data {
    mpc_decoder decoder;
    mpc_reader reader;
    mpc_streaminfo info;
    mpc_int32_t size;

    int samples;

    MPC_SAMPLE_FORMAT buffer[MPC_BUFFER_SIZE];
    int buf_end;
    int buf_pos;
};

/*
 * Musepack reader implementation
 */
mpc_int32_t
mpc_read_impl(void *data, void *ptr, mpc_int32_t size)
{
    struct audio_file *f = (struct audio_file *) data;
    return fread(ptr, 1, size, f->fp);
}

mpc_bool_t
mpc_seek_impl(void *data, mpc_int32_t offset)
{
    struct audio_file *f = (struct audio_file *) data;
    return f->stream ? 0 : !fseek(f->fp, offset, SEEK_SET);
}

mpc_int32_t
mpc_tell_impl(void *data)
{
    struct audio_file *f = (struct audio_file *) data;
    return ftell(f->fp);
}

mpc_int32_t
mpc_get_size_impl(void *data)
{
    struct musepack_drv_data *d = (struct musepack_drv_data *)
                                  ((struct audio_file *)data)->drv_data;
    return d->size;
}

mpc_bool_t
mpc_canseek_impl(void *data)
{
    struct audio_file *f = (struct audio_file *) data;
    return !f->stream;
}

/*
 * Public API
 */

int
musepack_open(struct audio_file *fd, const char *ext)
{
    struct musepack_drv_data *data;

    data = g_slice_new(struct musepack_drv_data);
    fd->drv_data = (void *)data;
    data->buf_pos = 0;
    data->buf_end = 0;

    data->samples = 0;

    /* setup the mpc reader functions */
    data->reader.read     = mpc_read_impl;
    data->reader.seek     = mpc_seek_impl;
    data->reader.tell     = mpc_tell_impl;
    data->reader.get_size = mpc_get_size_impl;
    data->reader.canseek  = mpc_canseek_impl;
    data->reader.data     = fd;

    if (!fd->stream) {
        fseek(fd->fp, 0, SEEK_END);
        data->size = ftell(fd->fp);
        fseek(fd->fp, 0, SEEK_SET);
    } else {
        /* no streaming support at the moment */
        return -1;
    }

    mpc_streaminfo_init(&data->info);
    if (mpc_streaminfo_read(&data->info, &data->reader) != ERROR_CODE_OK) {
        /* not a musepack file */
        g_slice_free(struct musepack_drv_data, data);
        return -1;
    }

    fd->srate    = data->info.sample_freq;
    fd->channels = data->info.channels;
    fd->time_len = mpc_streaminfo_get_length(&data->info);

    /* initialize the mpc decoder */
    mpc_decoder_setup(&data->decoder, &data->reader);
    if (!mpc_decoder_initialize(&data->decoder, &data->info)) {
        g_slice_free(struct musepack_drv_data, data);
        return -1;
    }

    return 0;
}

void
musepack_close(struct audio_file* fd)
{
    struct musepack_drv_data *data = (struct musepack_drv_data *)fd->drv_data;
    g_slice_free(struct musepack_drv_data, data);
}

#ifdef MPC_FIXED_POINT
#define TO_INT16(x) x >> 16
#else
/*
 * looks like sometimes libmpcdec returns floats slightly over 1.0f, so I do
 * not use 0x7FFF as a scale to avoid some audio artifacts
 */
#define TO_INT16(x) x * (0x6FFF)
#endif

size_t
musepack_read(struct audio_file* fd, int16_t *buf, size_t len)
{
    struct musepack_drv_data *data = (struct musepack_drv_data *)fd->drv_data;
    int copied = 0;
    size_t rlen;

    /* copy already decoded data in the buffer */
    while (data->buf_pos < data->buf_end)
        buf[copied++] = TO_INT16(data->buffer[data->buf_pos++]);

    data->buf_pos = 0;
    data->buf_end = 0;

    /* decode some new data */
    while (data->buf_end < len - copied) {
        rlen = mpc_decoder_decode(&data->decoder, 
                                  &data->buffer[data->buf_end], 
                                  0, 0);
        if (rlen <= 0)
            break;
        data->buf_end += rlen * fd->channels;
    }

    /* convert samples to 16 bit */
    while (copied < len && data->buf_pos < data->buf_end)
        buf[copied++] = TO_INT16(data->buffer[data->buf_pos++]);

    /* update current time */
    data->samples += copied / fd->channels; 
    fd->time_cur = data->samples / (float)fd->srate;

    return copied;
}

void
musepack_seek(struct audio_file* fd, int len, int rel)
{
    struct musepack_drv_data *data = (struct musepack_drv_data *)fd->drv_data;
    int pos = len;
    if (rel) {
        pos += fd->time_cur;
        data->samples += len * fd->srate;
    }
    else
        data->samples = pos * fd->srate;

    mpc_decoder_seek_seconds(&data->decoder, pos);
}

