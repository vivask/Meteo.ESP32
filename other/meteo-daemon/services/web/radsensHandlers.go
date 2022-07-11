package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) radsensHandler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "radsens.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)

}

func (web *Web) getRadsensPerDay(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetRadsensPerDay(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getRadsensPerWeek(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetRadsensPerWeek(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getRadsensPerMonth(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetRadsensPerMonth(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getRadsensPerYear(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetRadsensPerYear(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getNotSyncRadsens(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetNotSyncRadsens(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) addSyncRadsens(w http.ResponseWriter, r *http.Request) {
	var readings []data.Radsens
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddSyncRadsens(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) syncRadsens(w http.ResponseWriter, r *http.Request) {
	err := web.repo.SyncRadsens(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceRadsens(w http.ResponseWriter, r *http.Request) {
	var readings []data.Radsens
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceRadsens(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) lockRadsens(w http.ResponseWriter, r *http.Request) {
	err := web.repo.LockRadsens(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) unlockRadsens(w http.ResponseWriter, r *http.Request) {
	err := web.repo.UnlockRadsens(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
