/*-
 * ng_tcpmss.c
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2004, Alexey Popov <lollypop@flexuser.ru>
 * Copyright (c) 2016 Rozhuk Ivan <rozhuk.im@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
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
 *
 * This software includes fragments of the following programs:
 *	tcpmssd		Ruslan Ermilov <ru@FreeBSD.org>
 */

/*
 * This node is netgraph tool for workaround of PMTUD problem. It acts
 * like filter for IP packets. If configured, it reduces MSS of TCP SYN
 * packets.
 *
 * Configuration can be done by sending NGM_TCPMSS_CONFIG message. The
 * message sets filter for incoming packets on hook 'inHook'. Packet's
 * TCP MSS field is lowered to 'maxMSS' parameter and resulting packet
 * is sent to 'outHook'.
 *
 * XXX: statistics are updated not atomically, so they may broke on SMP.
 */

#include "opt_inet.h"
#include "opt_inet6.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#endif
#ifdef INET6
#include <net/vnet.h>
#include <netinet/ip6.h>
#include <netinet6/ip6_var.h>
#endif
#include <netinet/tcp.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_parse.h>
#include <netgraph/ng_tcpmss.h>

#ifdef NG_SEPARATE_MALLOC
MALLOC_DEFINE(M_NETGRAPH_TCPMSS, "netgraph_tcpmss", "netgraph tcpmss node");
#else
#define M_NETGRAPH_TCPMSS M_NETGRAPH
#endif


/* Per hook info. */
typedef struct {
	hook_p				outHook;
	struct ng_tcpmss_hookstat	stats;
	uint32_t			ip_offset;
	uint16_t			maxMSS6;
} *hpriv_p;

/* Netgraph methods. */
static ng_constructor_t	ng_tcpmss_constructor;
static ng_rcvmsg_t	ng_tcpmss_rcvmsg;
static ng_newhook_t	ng_tcpmss_newhook;
static ng_rcvdata_t	ng_tcpmss_rcvdata;
static ng_disconnect_t	ng_tcpmss_disconnect;

/* Parse type for struct ng_tcpmss_hookstat. */
static const struct ng_parse_struct_field ng_tcpmss_hookstat_type_fields[]
	= NG_TCPMSS_HOOKSTAT_INFO;
static const struct ng_parse_type ng_tcpmss_hookstat_type = {
	&ng_parse_struct_type,
	&ng_tcpmss_hookstat_type_fields
};

/* Parse type for struct ng_tcpmss_config. */
static const struct ng_parse_struct_field ng_tcpmss_config_type_fields[]
	= NG_TCPMSS_CONFIG_INFO;
static const struct ng_parse_type ng_tcpmss_config_type = {
	&ng_parse_struct_type,
	ng_tcpmss_config_type_fields
};

/* List of commands and how to convert arguments to/from ASCII. */
static const struct ng_cmdlist ng_tcpmss_cmds[] = {
	{
		NGM_TCPMSS_COOKIE,
		NGM_TCPMSS_GET_STATS,
		"getstats",
		&ng_parse_hookbuf_type,
		&ng_tcpmss_hookstat_type
	}, {
		NGM_TCPMSS_COOKIE,
		NGM_TCPMSS_CLR_STATS,
		"clrstats",
		&ng_parse_hookbuf_type,
		NULL
	}, {
		NGM_TCPMSS_COOKIE,
		NGM_TCPMSS_GETCLR_STATS,
		"getclrstats",
		&ng_parse_hookbuf_type,
		&ng_tcpmss_hookstat_type
	}, {
		NGM_TCPMSS_COOKIE,
		NGM_TCPMSS_CONFIG,
		"config",
		&ng_tcpmss_config_type,
		NULL
	},
	{ 0 }
};

/* Netgraph type descriptor. */
static struct ng_type ng_tcpmss_typestruct = {
	.version =	NG_ABI_VERSION,
	.name =		NG_TCPMSS_NODE_TYPE,
	.constructor =	ng_tcpmss_constructor,
	.rcvmsg =	ng_tcpmss_rcvmsg,
	.newhook =	ng_tcpmss_newhook,
	.rcvdata =	ng_tcpmss_rcvdata,
	.disconnect =	ng_tcpmss_disconnect,
	.cmdlist =	ng_tcpmss_cmds,
};

NETGRAPH_INIT(tcpmss, &ng_tcpmss_typestruct);


static inline int
m_chk(struct mbuf **mp, uint32_t len)
{
	if ((*mp)->m_pkthdr.len < len)
		return (EINVAL);
	if ((*mp)->m_len < len &&
	    NULL == ((*mp) = m_pullup((*mp), len)))
		return (ENOBUFS);
	return (0);
}

/*
 * Code from tcpmssd.
 */
/*-
 * The following macro is used to update an
 * internet checksum.  "acc" is a 32-bit
 * accumulation of all the changes to the
 * checksum (adding in old 16-bit words and
 * subtracting out new words), and "cksum"
 * is the checksum value to be updated.
 */
#define TCP_ADJUST_CHECKSUM(acc, cksum) do {		\
	acc += cksum;					\
	if (0 > acc) {					\
		acc = -acc;				\
		acc = ((acc >> 16) + (acc & 0xffff));	\
		acc += (acc >> 16);			\
		cksum = ((uint16_t)~acc);		\
	} else {					\
		acc = ((acc >> 16) + (acc & 0xffff));	\
		acc += (acc >> 16);			\
		cksum = ((uint16_t)acc);		\
	}						\
} while (0);

static int
tcp_mss_correct(struct tcphdr *tc, uint32_t hlen, uint16_t maxmss, int flags)
{
	int res = 0, accumulate;
	uint32_t olen;
	uint16_t sum;
	uint8_t *opt_ptr, opt, optlen;

	for (olen = (hlen - sizeof(struct tcphdr)), opt_ptr = (uint8_t*)(tc + 1);
	     0 < olen;
	     olen -= optlen, opt_ptr += optlen) {
		opt = opt_ptr[0];
		if (TCPOPT_EOL == opt)
			break;
		if (TCPOPT_NOP == opt) {
			optlen = 1;
			continue;
		}
		optlen = opt_ptr[1];
		if (0 == optlen || optlen > olen)
			break;
		if (TCPOPT_MAXSEG != opt ||
		    TCPOLEN_MAXSEG != optlen)
			continue;
		accumulate = be16dec((opt_ptr + 2));
		if (accumulate <= maxmss)
			continue;
		be16enc((opt_ptr + 2), maxmss);
		res ++;
		if (0 != (CSUM_TCP & flags))
			continue;
		accumulate -= maxmss;
		sum = be16dec(&tc->th_sum);
		TCP_ADJUST_CHECKSUM(accumulate, sum);
		be16enc(&tc->th_sum, sum);
	}
	return (res);
}

/*
 * Node constructor. No special actions required.
 */
static int
ng_tcpmss_constructor(node_p node)
{
	return (0);
}

/*
 * Add a hook. Any unique name is OK.
 */
static int
ng_tcpmss_newhook(node_p node, hook_p hook, const char *name)
{
	hpriv_p priv;

	priv = malloc(sizeof(*priv), M_NETGRAPH_TCPMSS, (M_NOWAIT | M_ZERO));
	if (priv == NULL)
		return (ENOMEM);

	NG_HOOK_SET_PRIVATE(hook, priv);

	return (0);
}

/*
 * Receive a control message.
 */
static int
ng_tcpmss_rcvmsg(node_p node, item_p item, hook_p lasthook)
{
	int error = 0;
	struct ng_mesg *msg, *resp = NULL;
	hook_p in, out;
	hpriv_p priv;
	struct ng_tcpmss_config *set;

	NGI_GET_MSG(item, msg);

	switch (msg->header.typecookie) {
	case NGM_TCPMSS_COOKIE:
		switch (msg->header.cmd) {
		case NGM_TCPMSS_GET_STATS:
		case NGM_TCPMSS_CLR_STATS:
		case NGM_TCPMSS_GETCLR_STATS:
			/* Check that message is long enough. */
			if (NG_HOOKSIZ != msg->header.arglen) {
				error = EINVAL;
				break;
			}

			/* Find this hook. */
			in = ng_findhook(node, (char*)msg->data);
			if (NULL == in) {
				error = ENOENT;
				break;
			}
			priv = NG_HOOK_PRIVATE(in);

			/* Create response. */
			if (NGM_TCPMSS_CLR_STATS != msg->header.cmd) {
				NG_MKRESPONSE(resp, msg,
				    sizeof(struct ng_tcpmss_hookstat), M_NOWAIT);
				if (NULL == resp) {
					error = ENOMEM;
					break;
				}
				bcopy(&priv->stats, resp->data,
				    sizeof(struct ng_tcpmss_hookstat));	
			}

			if (NGM_TCPMSS_GET_STATS != msg->header.cmd) {
				bzero(&priv->stats,
				    sizeof(struct ng_tcpmss_hookstat));
			}
			break;
		case NGM_TCPMSS_CONFIG:
			/* Check that message is long enough. */
			if (sizeof(struct ng_tcpmss_config) !=
			    msg->header.arglen) {
				error = EINVAL;
				break;
			}

			set = (struct ng_tcpmss_config*)msg->data;
			in = ng_findhook(node, set->inHook);
			out = ng_findhook(node, set->outHook);
			if (NULL == in || NULL == out) {
				error = ENOENT;
				break;
			}
			priv = NG_HOOK_PRIVATE(in);

			/* Configure MSS. */
			priv->outHook = out;
			if (0 != set->MTU) { /* Auto calc MSS from MTU */
#ifdef INET
				priv->stats.maxMSS = (set->MTU -
				    (sizeof(struct ip) + sizeof(struct tcphdr)));
#endif
#ifdef INET6
				priv->maxMSS6 = (set->MTU -
				    (sizeof(struct ip6_hdr) + sizeof(struct tcphdr)));
#endif
			}
			/* User defined MSS has more priority. */
			if (0 != set->maxMSS) {
				priv->stats.maxMSS = set->maxMSS;
			}
			if (0 != set->maxMSS6) {
				priv->maxMSS6 = set->maxMSS6;
			}
			priv->ip_offset = set->ip_offset;
			break;
		default:
			error = EINVAL;
			break;
		}
		break;
	default:
		error = EINVAL;
		break;
	}

	NG_RESPOND_MSG(error, node, item, resp);
	NG_FREE_MSG(msg);

	return (error);
}

/*
 * Receive data on a hook, and hack MSS.
 *
 */
static int
ng_tcpmss_rcvdata(hook_p hook, item_p item)
{
	int error = 0;
	hpriv_p priv = NG_HOOK_PRIVATE(hook);
	struct mbuf *m = NULL;
	struct tcphdr *tcp;
	uint32_t pullup_len, pkt_len, ip_off, ip_len, ip_hlen, tcp_hlen;
	uint16_t maxmss;
	uint8_t ip_v;
#ifdef INET
	struct ip *ip4;
#endif
#ifdef INET6
	int proto, last_off;
	struct ip6_hdr *ip6;
#endif

	NGI_GET_M(item, m);

	/* Drop packets if filter is not configured on this hook. */
	if (NULL == priv->outHook) {
		error = ENETDOWN;
		goto drop;
	}

	/* Do checks to verify assumptions made by code past this point. */
	if (0 == (M_PKTHDR & m->m_flags)) {
		error = EINVAL;
		goto drop;
	}

	/* Update stats on incoming hook. */
	pkt_len = m->m_pkthdr.len;
	priv->stats.Octets += pkt_len;
	priv->stats.Packets ++;

	/* Check whether we configured to fix MSS. */
	if (0 == priv->stats.maxMSS)
		goto fwd_pkt;

	/* Get IP version. */
	ip_off = priv->ip_offset;
	if (0 != (error = m_chk(&m, (ip_off + 1))))
		goto mchk_err;
	pullup_len = ip_off;
	ip_v = (*(mtod(m, uint8_t*) + ip_off));
#if BYTE_ORDER == LITTLE_ENDIAN
	ip_v = (ip_v >> 4);
#endif
#if BYTE_ORDER == BIG_ENDIAN
	ip_v &= 0x0f;
#endif

	/* Process IP. */
	switch (ip_v) {
#ifdef INET
	case IPVERSION:
		/* Pool up ip header. */
		pullup_len += sizeof(struct ip);
		if (0 != (error = m_chk(&m, pullup_len)))
			goto mchk_err;
		ip4 = (struct ip*)mtodo(m, ip_off);
		ip_len = ntohs(ip4->ip_len);
		ip_hlen = (ip4->ip_hl << 2);

		/* Basic packet checks. */
		if (sizeof(struct ip) > ip_hlen ||
		    ip_len < ip_hlen ||
		    (ip_off + ip_len) > pkt_len)
			goto drop; /* Bad packet. */
		/*
		 * Bypass:
		 * - non TCP;
		 * - fragments with offset (non first frag);
		 */
		if (IPPROTO_TCP != ip4->ip_p ||
		    0 != (htons(IP_OFFMASK) & ip4->ip_off))
			goto fwd_pkt;
		/* In case of IP header with options, we haven't pulled up enough. */
		pullup_len += (ip_hlen - sizeof(struct ip));
		maxmss = priv->stats.maxMSS;
		break;
#endif
#ifdef INET6
	case (IPV6_VERSION >> 4):
		/* Pool up ip header. */
		pullup_len += sizeof(struct ip6_hdr);
		if (0 != (error = m_chk(&m, pullup_len)))
			goto mchk_err;
		ip6 = (struct ip6_hdr*)mtodo(m, ip_off);
		ip_len = ntohs(ip6->ip6_plen);

		/* Basic packet checks. */
		if ((ip_off + ip_len) > pkt_len)
			goto drop; /* Bad packet. */

		/* Get IPv6 paload. */
		last_off = ip6_lasthdr(m, ip_off, IPPROTO_IPV6, &proto);
		if (0 > last_off)
			goto drop; /* Bad packet. */
		ip_hlen = (last_off - ip_off);

		/* Bypass non TCP. */
		if (IPPROTO_TCP != proto)
			goto fwd_pkt;
		/* In case of IP header with options, we haven't pulled up enough. */
		pullup_len += (ip_hlen - sizeof(struct ip6_hdr));
		maxmss = priv->maxMSS6;
		break;
#endif
	default:
		goto fwd_pkt;
	}

	/* Process TCP. */
	/* Pool up TCP header. */
	pullup_len += sizeof(struct tcphdr);
	if (0 != (error = m_chk(&m, pullup_len)))
		goto mchk_err;
	tcp = (struct tcphdr*)mtodo(m, (ip_off + ip_hlen));

	/* Check TCP header length. */
	tcp_hlen = (tcp->th_off << 2);
	if (sizeof(struct tcphdr) > tcp_hlen ||
	    (pkt_len - ip_hlen) < tcp_hlen)
		goto drop; /* Bad packet. */

	/* Check is SYN packet and has options. */
	if (0 == (TH_SYN & tcp->th_flags) ||
	    sizeof(struct tcphdr) == tcp_hlen)
		goto fwd_pkt;

	/* Update SYN stats. */
	priv->stats.SYNPkts ++;

	/* Pool up TCP header + header options. */
	pullup_len += (tcp_hlen - sizeof(struct tcphdr));
	if (0 != (error = m_chk(&m, pullup_len)))
		goto mchk_err;
	tcp = (struct tcphdr*)mtodo(m, (ip_off + ip_hlen));

	/* Fix MSS and update stats. */
	if (tcp_mss_correct(tcp, tcp_hlen, maxmss, m->m_pkthdr.csum_flags)) {
		priv->stats.FixedPkts ++;
	}

fwd_pkt:
	/* Deliver frame out destination hook. */
	NG_FWD_NEW_DATA(error, item, priv->outHook, m);
	return (error);

drop:
	m_freem(m);
	NG_FREE_ITEM(item);
	return (error);

mchk_err:
	if (EINVAL == error)
		goto fwd_pkt; /* Bypass packet. */
	NG_FREE_ITEM(item);
	return (error);
}

/*
 * Hook disconnection.
 * We must check all hooks, since they may reference this one.
 */
static int
ng_tcpmss_disconnect(hook_p hook)
{
	node_p node = NG_HOOK_NODE(hook);
	hook_p hook2;
	hpriv_p priv;

	LIST_FOREACH(hook2, &node->nd_hooks, hk_hooks) {
		priv = NG_HOOK_PRIVATE(hook2);
		if (priv->outHook == hook) {
			priv->outHook = NULL;
		}
	}

	free(NG_HOOK_PRIVATE(hook), M_NETGRAPH);

	if (0 == NG_NODE_NUMHOOKS(NG_HOOK_NODE(hook))) {
		ng_rmnode_self(NG_HOOK_NODE(hook));
	}

	return (0);
}
