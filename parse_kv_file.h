/**
 * parse_kv_file.h - Parse 'Key=Value', ini-style file
 *
 * Copyright (c) 2019, David Imhoff <dimhoff.devel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __PARSE_KV_FILE_H__
#define __PARSE_KV_FILE_H__

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * parse_kv_file() callback function signature.
 *
 * NOTE: the variable _key_ and _value_ are only valid for the duration of this
 * function. If you want to store these values copy the whole string, not just
 * the pointer.
 */
typedef int (*parse_kv_cb_t)(const char *key, const char *value);

/**
 * Parse 'key=value' type file.
 *
 * Parse a text file containing key/value pairs. The format is similar to the
 * Microsoft Windows INI files but more limited. Lines starting with a '#' are
 * ignored. Leading and trailing white space are stripped from both key and
 * value.
 * 
 * For every key/value pair found the callback function is called. If the
 * callback returns a non-zero value parsing will be aborted and the value
 * returned by the callback is returned.
 *
 * @param fp		File pointer to read from
 * @param process_cb	Callback function that is invoked for very key value pair.
 *
 * @returns		0 on success, -1 on parsing error, or the error code
 *			returned by the process_cb() function.
 */
int parse_kv_file(FILE *fp, parse_kv_cb_t process_cb);

#ifdef __cplusplus
}
#endif

#endif // __PARSE_KV_FILE_H__
