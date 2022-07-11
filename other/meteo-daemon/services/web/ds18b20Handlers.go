package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) ds18b20Handler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "ds18b20.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) getDs18b20PerDay(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetDs18b20PerDay(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getDs18b20PerWeek(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetDs18b20PerWeek(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getDs18b20PerMonth(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetDs18b20PerMonth(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)

}

func (web *Web) getDs18b20PerYear(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetDs18b20PerYear(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getNotSyncDs18b20(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetNotSyncDs18b20(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) addSyncDs18b20(w http.ResponseWriter, r *http.Request) {
	var readings []data.Ds18b20
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddSyncDs18b20(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) syncDs18b20(w http.ResponseWriter, r *http.Request) {
	err := web.repo.SyncDs18b20(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceDs18b20(w http.ResponseWriter, r *http.Request) {
	var readings []data.Ds18b20
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceDs18b20(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) lockDs18b20(w http.ResponseWriter, r *http.Request) {
	err := web.repo.LockDs18b20(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) unlockDs18b20(w http.ResponseWriter, r *http.Request) {
	err := web.repo.UnlockDs18b20(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
