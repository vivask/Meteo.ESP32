package web

import (
	"fmt"
	"net/http"
)

func (web *Web) telegramSend(w http.ResponseWriter, r *http.Request) {
	var msg string
	err := FromJSON(&msg, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	err = web.tg.Message(msg)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) MiddlewareValidateTelegram(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		w.Header().Set("Content-Type", "application/json")

		if web.tg == nil {
			errServiceNotAvailable := fmt.Errorf("service Telegram not available")
			web.logger.Error(errServiceNotAvailable)
			w.WriteHeader(http.StatusBadRequest)
			ToJSON(errServiceNotAvailable, w)
			return
		}

		// call the next handler
		next.ServeHTTP(w, r)
	})
}
