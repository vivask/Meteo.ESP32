package metrics

import (
	"github.com/prometheus/client_golang/prometheus"
)

var (
	totalRequestsTcp = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "total",
		Help:      "total requests",

		ConstLabels: map[string]string{
			"type": "tcp",
		},
	}))

	totalRequestsUdp = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "total",
		Help:      "total requests",

		ConstLabels: map[string]string{
			"type": "udp",
		},
	}))

	totalRequestsFailed = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "failed",
		Help:      "failed requests",
	}))

	totalRequestsInternal = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "internal",
		Help:      "internal requests",
	}))

	totalRequestsBlocked = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "blocked",
		Help:      "blocked requests",
	}))

	totalRequestsSuccess = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "success",
		Help:      "success requests",
	}))

	totalRequestsLookup = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "google",
		Help:      "google requests",
	}))

	totalCacheHits = prometheus.NewCounter(prometheus.CounterOpts(prometheus.Opts{
		Namespace: "dns",
		Subsystem: "requests",
		Name:      "cache",
		Help:      "cached requests",
	}))
)

func (m *Metrics) TotalRequestsTcp() {
	//totalRequestsTcp.Inc()
}

func (m *Metrics) TotalRequestsUdp() {
	//totalRequestsUdp.Inc()
}

func (m *Metrics) TotalRequestsFailed() {
	//totalRequestsFailed.Inc()
}

func (m *Metrics) TotalRequestsInternal() {
	//totalRequestsInternal.Inc()
}

func (m *Metrics) TotalRequestsBlocked() {
	//totalRequestsBlocked.Inc()
}

func (m *Metrics) TotalRequestsSuccess() {
	//totalRequestsSuccess.Inc()
}

func (m *Metrics) TotalRequestsLookup() {
	//totalRequestsLookup.Inc()
}

func (m *Metrics) TotalCacheHits() {
	//totalCacheHits.Inc()
}
