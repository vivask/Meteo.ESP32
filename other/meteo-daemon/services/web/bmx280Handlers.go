package web

import (
	"context"
	"meteo-daemon/services/data"
	"net/http"
)

func (web *Web) bme280Handler(w http.ResponseWriter, r *http.Request) {
	web.render(w, r, "bme280.page.html", nil, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) getBmx280PerDayAvg(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerDayAvg(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerDayMin(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerDayMin(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerDayMax(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerDayMax(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerWeekAvg(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerWeekAvg(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerWeekMin(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerWeekMin(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerWeekMax(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerWeekMax(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerMonthAvg(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerMonthAvg(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerMonthMin(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerMonthMin(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerMonthMax(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerMonthMin(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerYearAvg(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerYearAvg(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerYearMin(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerYearMin(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) getBmx280PerYearMax(w http.ResponseWriter, r *http.Request) {
	var p data.Period
	err := FromJSON(&p, r.Body)
	if err != nil {
		web.logger.Error(err)
		return
	}
	data, err := web.repo.GetBmx280PerYearMax(context.Background(), p)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

/*func (web *Web) putBmx280(w http.ResponseWriter, r *http.Request) {
	var bmx280 []data.Bmx280
	err := FromJSON(&bmx280, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.PutBmx280(context.Background(), bmx280)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}*/

func (web *Web) getNotSyncBmx280(w http.ResponseWriter, r *http.Request) {
	data, err := web.repo.GetNotSyncBmx280(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(data, w)
}

func (web *Web) addSyncBmx280(w http.ResponseWriter, r *http.Request) {
	var readings []data.Bmx280
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddSyncBmx280(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) syncBmx280(w http.ResponseWriter, r *http.Request) {
	err := web.repo.SyncBmx280(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) replaceBmx280(w http.ResponseWriter, r *http.Request) {
	var readings []data.Bmx280
	err := FromJSON(&readings, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ReplaceBmx280(context.Background(), readings)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) lockBmx280(w http.ResponseWriter, r *http.Request) {
	err := web.repo.LockBmx280(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) unlockBmx280(w http.ResponseWriter, r *http.Request) {
	err := web.repo.UnlockBmx280(false)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}
