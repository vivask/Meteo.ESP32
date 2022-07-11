package scheduler

import (
	"context"
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/domain"
	"meteo-daemon/leader"
	"meteo-daemon/services/data"
	u "meteo-daemon/utils"
	"os"
	"time"

	"github.com/go-co-op/gocron"

	"github.com/sirupsen/logrus"
)

type Params struct {
	Hasp   domain.StartStopInterface
	Logger *logrus.Entry
	Repo   data.Repository
	Cli    *client.Client
	Items  map[string]SheduleItem
	Lead   *leader.Leader
}

type SheduleItem interface {
	Run(params map[string]string) error
	Terminate() error
}

type Scheduler struct {
	hasp      domain.StartStopInterface
	logger    *logrus.Entry
	logfile   *os.File
	config    *Config
	repo      data.Repository
	cli       *client.Client
	cron      *gocron.Scheduler
	jobs      map[uint32]*gocron.Job
	listToRun map[string]SheduleItem
	lead      *leader.Leader
}

type Options struct {
	item   SheduleItem
	param  map[string]string
	job    data.Jobs
	off    bool
	repeat bool
}

func New(cnf *Config, p *Params) *Scheduler {
	s := &Scheduler{
		hasp:      p.Hasp,
		logger:    p.Logger,
		config:    cnf,
		repo:      p.Repo,
		cli:       p.Cli,
		cron:      gocron.NewScheduler(time.Local),
		jobs:      map[uint32]*gocron.Job{},
		listToRun: p.Items,
		lead:      p.Lead,
	}

	//s.cron.SingletonMode()

	return s
}

func (s *Scheduler) getJobParams(job_id uint32) (params map[string]string, err error) {
	jobParams, err := s.repo.GetJobParamsById(context.Background(), job_id)
	if err != nil {
		return nil, fmt.Errorf("GetJobParamsById error: %w", err)
	}
	params = map[string]string{}
	for _, p := range jobParams {
		params[p.Name] = p.Value
	}
	return params, nil
}

func (s *Scheduler) jobFunc(o *Options) {
	err := o.item.Run(o.param)
	if err != nil {
		s.logger.Error("Job: [%v] executed error: %v", o.job.Note, err)
	} else {
		if o.job.Verbose != 0 {
			s.logger.Infof("Job [%v] executed success", o.job.Note)
		}
	}
	if o.off {
		if val, ok := s.jobs[o.job.ID]; ok {
			s.cron.RemoveByReference(val)
			delete(s.jobs, o.job.ID)
		} else {
			s.logger.Errorf("Cron ID: %d not found", o.job.ID)
			return
		}
		err = s.repo.DeactivateJob(context.Background(), o.job.ID, false)
		if err != nil {
			s.logger.Error(err)
		}
	}
	if o.repeat {
		err = s.ReloadJobs()
		if err != nil {
			s.logger.Errorf("reload jobs error: %v", err)
		}
	}
}

func (s *Scheduler) addJob(job *data.Jobs) (cronJob *gocron.Job, err error) {

	if job.Active == 0 {
		return nil, nil
	}

	params, err := s.getJobParams(job.ID)
	if err != nil {
		return nil, fmt.Errorf("getJobParams error: %w", err)
	}

	repeat := job.Value

	options := Options{
		item:   s.listToRun[job.TaskID],
		param:  params,
		job:    *job,
		off:    false,
		repeat: false,
	}

	dt := NewTime(job.Date, job.Time)
	if err != nil {
		return nil, fmt.Errorf("NeTime create error: %w", err)
	}

	switch job.Period.ID {
	case "one":
		options.off = true
		cronJob, err = s.createJob(dt, job, &options, s.cron.Days())
	case "sec":
		cronJob, err = s.createSimpleJob(dt, job, &options, s.cron.Every(repeat).Seconds())
	case "min":
		cronJob, err = s.createSimpleJob(dt, job, &options, s.cron.Every(repeat).Minutes())
	case "hour":
		cronJob, err = s.createSimpleJob(dt, job, &options, s.cron.Every(repeat).Hours())
	case "day":
		cronJob, err = s.createJob(dt, job, &options, s.cron.Every(repeat).Days())
	case "week":
		cronJob, err = s.createJob(dt, job, &options, s.cron.Every(repeat).Weeks())
	case "month":
		cronJob, err = s.createJob(dt, job, &options, s.cron.Every(repeat).Month())
	case "day_of_week":
		switch job.Day.ID {
		case 1:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Monday())
		case 2:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Tuesday())
		case 3:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Wednesday())
		case 4:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Thursday())
		case 5:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Friday())
		case 6:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Saturday())
		case 7:
			cronJob, err = s.createDayOfWeekJob(dt, repeat, job, &options, s.cron.Every(1).Sunday())
		default:
			err = fmt.Errorf("Invalid day of the week: %v", job.Day.ID)
			return nil, err
		}
	default:
		err = fmt.Errorf("Unknown period id: %s", job.Period.ID)
		return nil, err
	}

	return cronJob, err
}

func (s *Scheduler) createSimpleJob(dt *DateTime, job *data.Jobs, options *Options, fn *gocron.Scheduler) (cronJob *gocron.Job, err error) {
	if dt.IsZero() {
		cronJob, err = fn.Tag(job.Note).Do(s.jobFunc, options)
	} else {
		stamp, err := dt.Stamp()
		if err != nil {
			return nil, fmt.Errorf("Stamp error: %w", err)
		}
		cronJob, err = fn.StartAt(stamp).Tag(job.Note).Do(s.jobFunc, options)
	}

	return
}

func (s *Scheduler) createJob(dt *DateTime, job *data.Jobs, options *Options, fn *gocron.Scheduler) (cronJob *gocron.Job, err error) {
	if dt.IsZero() {
		cronJob, err = fn.Tag(job.Note).Do(s.jobFunc, &options)
	} else {
		if dt.TimeOnly() {
			cronJob, err = fn.At(dt.Time()).Tag(job.Note).Do(s.jobFunc, options)
		} else {
			stamp, err := dt.Stamp()
			if err != nil {
				return nil, fmt.Errorf("Stamp error: %w", err)
			}
			cronJob, err = fn.StartAt(stamp).Tag(job.Note).Do(s.jobFunc, options)
		}
	}

	return
}

func (s *Scheduler) createDayOfWeekJob(dt *DateTime, repeat int, job *data.Jobs, options *Options, fn *gocron.Scheduler) (cronJob *gocron.Job, err error) {

	if repeat < 0 || repeat > 4 {
		return nil, fmt.Errorf("number [%d] week of month incorrect, expected 0-4", repeat)
	}

	if dt.IsZero() {
		dt.ts = "00:00:00"
	}

	options.repeat = false

	if dt.TimeOnly() {
		if repeat < 1 {
			cronJob, err = fn.At(dt.Time()).Tag(job.Note).Do(s.jobFunc, options)
			//weekDay, _ := cronJob.Weekday()
			//s.logger.Debugf("Job [%s] Start On: %v At: %v", job.Note, weekDay, dt.Time())
		} else {
			stamp, err := s.getNextDateStartJob(int(job.Day.ID), repeat, dt.Time(), job.Note)
			if err != nil {
				return nil, fmt.Errorf("getNextDateStartJob error: %w", err)
			}
			options.repeat = true
			cronJob, err = fn.StartAt(stamp).Tag(job.Note).Do(s.jobFunc, options)
		}
	} else {
		if repeat < 1 {
			stamp, err := dt.Stamp()
			if err != nil {
				return nil, fmt.Errorf("Stamp error: %w", err)
			}
			//s.logger.Debugf("Job [%s] Start At: %v", job.Note, stamp)
			cronJob, err = fn.StartAt(stamp).Tag(job.Note).Do(s.jobFunc, options)
		} else {
			stamp, err := s.getNextDateStartJob(int(job.Day.ID), repeat, dt.Time(), job.Note)
			if err != nil {
				return nil, fmt.Errorf("getNextDateStartJob error: %w", err)
			}
			options.repeat = true
			//s.logger.Debugf("Job [%s] Start At: %v", job.Note, stamp)
			cronJob, err = fn.StartAt(stamp).Tag(job.Note).Do(s.jobFunc, options)
		}
	}

	return
}

func (s *Scheduler) selectTask(job *data.Jobs) error {
	if job.Executor == "Master" && !s.lead.IsMaster() {
		s.logger.Warnf("can't run job [%s] as Slave, need Master", job.Note)
		return nil
	}
	if job.Executor == "Slave" && s.lead.IsMaster() {
		s.logger.Warnf("can't run job [%s] as Master, need Slave", job.Note)
		return nil
	}
	if job.Executor == "Leader" && !s.lead.IsLeader() {
		s.logger.Warnf("can't run job [%s], need Leader", job.Note)
		return nil
	}

	cronJob, err := s.addJob(job)
	if err != nil || cronJob == nil {
		return fmt.Errorf("addJob error: %w", err)
	}
	s.jobs[job.ID] = cronJob
	return nil
}

func (s *Scheduler) updateJob(job_id uint32) error {
	job, err := s.repo.GetJob(context.Background(), job_id)
	if err != nil {
		return fmt.Errorf("GetJob error: %w", err)
	}
	//activate job
	if job.Active == 1 {
		return s.exeJob(job_id)
	}
	//deactivate job
	if val, ok := s.jobs[job_id]; ok {
		s.cron.RemoveByReference(val)
	} else {
		return fmt.Errorf("Unknown cron job id: %d", job_id)
	}
	return nil
}

func (s *Scheduler) removeJob(job_id uint32) error {
	job, err := s.repo.GetJob(context.Background(), job_id)
	if err != nil {
		return fmt.Errorf("GetJob error: %w", err)
	}
	if job.Active == 1 {
		if val, ok := s.jobs[job_id]; ok {
			s.cron.RemoveByReference(val)
			delete(s.jobs, job_id)
		} else {
			return fmt.Errorf("Unknown cron job id: %d", job_id)
		}
	}
	return nil
}

func (s *Scheduler) runJob(job_id uint32) (err error) {
	job, err := s.repo.GetJob(context.Background(), job_id)
	if err != nil {
		return fmt.Errorf("GetJob error: %w", err)
	}
	if task, ok := s.listToRun[job.Task.ID]; ok {
		params, err := s.getJobParams(job_id)
		if err != nil {
			return err
		}
		err = task.Run(params)
		if err != nil {
			return err
		}
	} else {
		err = fmt.Errorf("Unknown task id: %s", job.Task.ID)
	}

	return err
}

func (s *Scheduler) ReloadJobs() error {
	for _, cronJob := range s.jobs {
		s.cron.RemoveByReference(cronJob)
	}
	s.cron.Update()

	s.jobs = make(map[uint32]*gocron.Job)
	err := s.exeJobs()
	if err != nil {
		return fmt.Errorf("exeJobs error: %w", err)
	}
	s.cron.Update()
	s.LogActiveJobs()
	s.logger.Debug("Jobs reloaded success")
	return nil
}

func (s *Scheduler) exeJobs() error {

	jobs, err := s.repo.GetAllActiveJobs(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllActiveJobs error: %w", err)
	}
	for _, job := range jobs {
		err = s.selectTask(&job)
		if err != nil {
			s.repo.DeactivateJob(context.Background(), job.ID, false)
			s.logger.Errorf("can't start job [%s] error: %v", job.Note, err)
		}
	}
	return nil
}

func (s *Scheduler) exeJob(job_id uint32) error {
	job, err := s.repo.GetJob(context.Background(), job_id)
	if err != nil {
		return fmt.Errorf("GetJob error: %w", err)
	}
	err = s.selectTask(job)
	if err != nil {
		return fmt.Errorf("selectTask error: %w", err)
	}
	return nil
}

func (Scheduler) StartNum() int {
	return 9
}

func (s *Scheduler) Enabled() bool {
	return s.config.Active
}

func (s *Scheduler) IsRun() bool {
	return s.hasp.IsRun()
}

func (s *Scheduler) Start() error {
	if !s.hasp.Start() {
		return fmt.Errorf("%s:failed to start", s.config.Title)
	}

	err := s.exeJobs()

	if err != nil {
		s.logger.Error(err)
	} else {
		go s.cron.StartAsync()
		s.logger.Debugf("%s: success started", s.config.Title)
	}

	s.LogActiveJobs()

	return err
}

func (s *Scheduler) Stop() error {
	if !s.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", s.config.Title)
	}

	s.cron.Stop()
	s.logger.Debugf("%s: success stoped", s.config.Title)

	if s.logfile != nil {
		s.logfile.Close()
	}

	return nil
}

func (s *Scheduler) InitLog(logDir string, file *os.File) error {
	f, err := u.InitLogrus(logDir, s.config.LogFile, s.config.LogLevel, file, s.logger)
	if err != nil {
		return err
	}
	s.logfile = f
	return nil
}

func (s *Scheduler) IsImplemented(task_id string) error {
	if _, ok := s.listToRun[task_id]; !ok {
		return fmt.Errorf("Task with id: '%s' is not implemented", task_id)
	}
	return nil
}

func (s *Scheduler) JobRemove(job_id uint32) error {
	err := s.ReloadJobs()
	if err != nil {
		return fmt.Errorf("reload jobs error: %w", err)
	}
	_, err = s.cli.PutExt("/verify/schedule/rest/job/reload", nil)
	if err != nil {
		return fmt.Errorf("remote reload jobs fail: %w", err)
	}
	s.logger.Debug("Job removed")
	return nil
}

func (s *Scheduler) JobUpdate(job_id uint32) error {
	err := s.ReloadJobs()
	if err != nil {
		return fmt.Errorf("reload jobs error: %w", err)
	}
	_, err = s.cli.PutExt("/verify/schedule/rest/job/reload", nil)
	if err != nil {
		return fmt.Errorf("remote reload jobs fail: %w", err)
	}
	s.logger.Debug("Job updated")
	return nil
}

func (s *Scheduler) JobCreate(job_id uint32) error {
	err := s.ReloadJobs()
	if err != nil {
		return fmt.Errorf("reload jobs error: %w", err)
	}
	_, err = s.cli.PutExt("/verify/schedule/rest/job/reload", nil)
	if err != nil {
		return fmt.Errorf("remote reload jobs fail: %w", err)
	}
	s.logger.Debug("Job created")
	return nil
}

func (s *Scheduler) JobRun(job_id uint32) error {
	err := s.runJob(job_id)
	if err != nil {
		return fmt.Errorf("can't execute job_d [%d], error: %w", job_id, err)
	}
	s.cron.Update()
	s.logger.Debug("Job executed")
	return nil
}

type Config struct {
	Title    string `toml:"Title"`
	Active   bool   `toml:"Active"`
	LogFile  string `toml:"LogFile"`
	LogLevel string `toml:"LogLevel"`
}
