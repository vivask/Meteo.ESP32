package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) logingHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "loging.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) getLoging(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetLoging(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&data, w)
}

func (web *Web) journalClear(w http.ResponseWriter, r *http.Request) {
	err := web.repo.JournalClear(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
