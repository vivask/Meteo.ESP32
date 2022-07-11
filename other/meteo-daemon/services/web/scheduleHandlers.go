package web

import (
	"context"
	"fmt"
	"meteo-daemon/services/data"
	"net/http"

	"github.com/gorilla/mux"
)

func (web *Web) sheduleHandler(w http.ResponseWriter, r *http.Request) {
	var err error
	var d struct {
		Jobs      []data.Jobs
		Tasks     []data.Tasks
		Periods   []data.Periods
		Days      []data.Days
		Executors []data.Executors
	}
	d.Jobs, err = web.repo.GetAllJobs(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	d.Tasks, err = web.repo.GetAllTasks(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	d.Periods, err = web.repo.GetAllPeriods(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	d.Days, err = web.repo.GetAllDays(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	d.Executors, err = web.repo.GetAllExecutors(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "schedule.page.html", &d, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) tasksHandler(w http.ResponseWriter, r *http.Request) {
	tasks, err := web.repo.GetAllTasks(context.Background())
	if err != nil {
		web.logger.Error(err)
	}
	web.render(w, r, "tasks.page.html", &tasks, false)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) cronHandler(w http.ResponseWriter, r *http.Request) {
	jobs := web.schd.GetCronJobs()
	web.render(w, r, "cron.page.html", &jobs, true)
	go web.metrics.ResponceCode(http.StatusOK)
}

func (web *Web) addJob(w http.ResponseWriter, r *http.Request) {
	var job data.Jobs
	err := FromJSON(&job, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddJob(context.Background(), job)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) editJob(w http.ResponseWriter, r *http.Request) {
	var job data.Jobs
	err := FromJSON(&job, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	job.ID, err = StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.EditJob(context.Background(), job)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) activateJob(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.ActivateJob(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) runJob(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.RunJob(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delJob(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.DelJob(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getJob(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	jobs, err := web.repo.GetJob(context.Background(), id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&jobs, w)
}

func (web *Web) getJobParams(w http.ResponseWriter, r *http.Request) {
	job_id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	jobs, err := web.repo.GetJobParams(context.Background(), job_id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&jobs, w)
}

func (web *Web) addTask(w http.ResponseWriter, r *http.Request) {
	var task data.Tasks
	err := FromJSON(&task, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.repo.AddTask(context.Background(), task)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) editTask(w http.ResponseWriter, r *http.Request) {
	var task data.Tasks
	err := FromJSON(&task, r.Body)
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	task.ID = mux.Vars(r)["id"]
	err = web.repo.EditTask(context.Background(), task)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) delTask(w http.ResponseWriter, r *http.Request) {
	err := web.repo.DelTask(context.Background(), mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) getTask(w http.ResponseWriter, r *http.Request) {
	jobs, err := web.repo.GetTask(context.Background(), mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&jobs, w)
}

func (web *Web) getTaskParams(w http.ResponseWriter, r *http.Request) {
	params, err := web.repo.GetTaskParams(context.Background(), mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(&params, w)
}

func (web *Web) isImplemented(w http.ResponseWriter, r *http.Request) {
	err := web.schd.IsImplemented(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) jobUpdate(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.schd.JobUpdate(id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) jobCreate(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.schd.JobCreate(id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) jobRun(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.schd.JobRun(id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) jobRemove(w http.ResponseWriter, r *http.Request) {
	id, err := StringToUint32(mux.Vars(r)["id"])
	if err != nil {
		web.logger.Error(err)
		ToJSON(GenericResponse(err), w)
		return
	}
	err = web.schd.JobRemove(id)
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) reloadJobs(w http.ResponseWriter, r *http.Request) {
	err := web.schd.ReloadJobs()
	if err != nil {
		web.logger.Error(err)
	}
	ToJSON(GenericResponse(err), w)
}

func (web *Web) MiddlewareValidateShedule(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {

		w.Header().Set("Content-Type", "application/json")

		if web.schd == nil {
			errServiceNotAvailable := fmt.Errorf("service Schedule not available")
			web.logger.Error(errServiceNotAvailable)
			w.WriteHeader(http.StatusBadRequest)
			ToJSON(errServiceNotAvailable, w)
			return
		}

		// call the next handler
		next.ServeHTTP(w, r)
	})
}
