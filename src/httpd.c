/*
 * Copyright (c) 2009, Manish Shakya,Real Time Solutions Pvt. Ltd.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
#include "uip.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "httpd.h"
static int handle_connection(struct simple_httpd_state *s);

void simple_httpd_init(void)
{
	uip_listen(HTONS(80));
}

void simple_httpd_appcall(void)
{
  struct simple_httpd_state *s = &(uip_conn->appstate);

  if(uip_connected()) {
		PSOCK_INIT(&s->p, NULL, 0);
  }

  handle_connection(s);
}

static int handle_connection(struct simple_httpd_state *s)
{
  PSOCK_BEGIN(&s->p);
  PSOCK_SEND_STR(&s->p, "HTTP/1.0 200 OK\r\n");
  PSOCK_SEND_STR(&s->p, "Content-Type: text/html\r\n");
  PSOCK_SEND_STR(&s->p, "\r\n");
  PSOCK_SEND_STR(&s->p, "<center><H1>Embedded Webserver Example</h1>");
  PSOCK_SEND_STR(&s->p, "<p><a href=\"http://www.sics.se/~adam/\">uIP TCP/IP Stack</a> LPC21XX Port By <a href=\"http://www.manishshakya.com.np\" >Manish Shakya</a> Website:http://www.manishshakya.com.np");
  PSOCK_SEND_STR(&s->p, "<p>Hardware Design By <a href=\"http://www.sandeeppaudel.com.np\" >Sandeep Paudel</a>");
  PSOCK_SEND_STR(&s->p, "<p><a href=\"http://www.rts.com.np\" >Real Time Solutions Pvt. Ltd.</a></center>");
  PSOCK_CLOSE(&s->p);
  PSOCK_END(&s->p);
}
