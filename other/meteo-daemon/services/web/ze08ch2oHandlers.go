package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) ze08ch2oHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "ze08ch2o.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) getZe08ch2oPerDay(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetZe08ch2oPerDay(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getZe08ch2oPerWeek(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetZe08ch2oPerWeek(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getZe08ch2oPerMonth(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetZe08ch2oPerMonth(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getZe08ch2oPerYear(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetZe08ch2oPerYear(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getNotSyncZe08ch2o(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetNotSyncZe08ch2o(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) addSyncZe08ch2o(w http.ResponseWriter, r *http.Request) {
	var readings []data.Ze08ch2o
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddSyncZe08ch2o(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) syncZe08ch2o(w http.ResponseWriter, r *http.Request) {
	err := web.repo.SyncZe08ch2o(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceZe08ch2o(w http.ResponseWriter, r *http.Request) {
	var readings []data.Ze08ch2o
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceZe08ch2o(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) lockZe08ch2o(w http.ResponseWriter, r *http.Request) {
	err := web.repo.LockZe08ch2o(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) unlockZe08ch2o(w http.ResponseWriter, r *http.Request) {
	err := web.repo.UnlockZe08ch2o(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
