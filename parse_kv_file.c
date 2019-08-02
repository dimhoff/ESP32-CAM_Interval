/**
 * parse_kv_file.c - Parse 'Key=Value', ini-style file
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
#include "parse_kv_file.h"

#include <stdio.h>
#include <string.h>

int parse_kv_file(FILE *fp, parse_kv_cb_t process_cb)
{
	char linebuf[128] = "";

	while (fgets(linebuf, sizeof(linebuf), fp) != NULL) {
		size_t len = strlen(linebuf);
		char *key = linebuf;
		char *value = NULL;
		char *endp = NULL;

		// Check for line to long
		if (linebuf[len - 1] != '\n') {
			// TODO: maybe different error code??
			return -1;
		}
		len--;
		linebuf[len] = '\0';

		if (len == 0) continue;

		// Ignore Windows line endings
		if (linebuf[len - 1] == '\r') {
			len--;
			linebuf[len] = '\0';
		}

		if (len == 0) continue;

		// Strip leading spaces
		while (key[0] == ' ') {
			key++;
			len--;
		}

		if (len == 0) continue;

		// Ignore comments
		if (key[0] == '#') continue;

		// Seperate Key/Value
		value = key;
		strsep(&value, "=");
		if (value == NULL) {
			return -1;
		}

		// Strip trailing spaces of key
		len = strlen(key);
		endp = &key[len - 1];
		while (*endp == ' ' && len > 0) {
			endp--;
			len--;
		}
		key[len] = '\0';

		if (len == 0) {
			return -1;
		}

		// Strip leading spaces of value
		while (value[0] == ' ') {
			value++;
		}

		// Strip trailing spaces of value
		len = strlen(value);
		endp = &value[len - 1];
		while (*endp == ' ' && len > 0) {
			endp--;
			len--;
		}
		value[len] = '\0';

		// Run call back for key/value pair
		int err = process_cb(key, value);
		if (err != 0) {
			return err;
		}
	}

	return 0;
}
