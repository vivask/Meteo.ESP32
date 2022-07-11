package web

import (
	"fmt"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) syncRouterZones(w http.ResponseWriter, r *http.Request) {
	hosts := map[string]string{}
	err := FromJSON(&hosts, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.mk.SyncZones(hosts)
	ToJSON(GenericResponse(err), w)
}

func (web *Web) putHostToVpn(w http.ResponseWriter, r *http.Request) {
	var host data.ToVpnManual
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.mk.PutHostToVpn(host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) removeHostFromVpn(w http.ResponseWriter, r *http.Request) {
	var host data.ToVpnManual
	err := FromJSON(&host, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}

	err = web.mk.RemoveHostFromVpn(host)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) MiddlewareValidateMikrotiks(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		w.Header().Set("Content-Type", "application/json")

		if web.mk == nil {
			errServiceNotAvailable := fmt.Errorf("service Mikrotiks not available")
			web.logger.Error(errServiceNotAvailable)
			w.WriteHeader(http.StatusBadRequest)
			ToJSON(errServiceNotAvailable, w)
			return
		}

		// call the next handler
		next.ServeHTTP(w, r)
	})
}
