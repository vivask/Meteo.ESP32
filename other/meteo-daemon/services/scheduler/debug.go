package scheduler

import "meteo-daemon/services/data"

func (s *Scheduler) LogActiveJobs() {
	for _, job := range s.cron.Jobs() {
		switch {
		case !job.ScheduledTime().IsZero():
			for _, tag := range job.Tags() {
				s.logger.Debugf("JOB [%s] ScheduledTime: %v", tag, job.ScheduledTime())
			}
		case len(job.ScheduledAtTime()) != 0:
			for _, tag := range job.Tags() {
				s.logger.Debugf("JOB [%s] ScheduledAtTime: %v", tag, job.ScheduledAtTime())
			}
		}
	}
}

func (s *Scheduler) GetCronJobs() (jobs []data.CronJobs) {
	for _, job := range s.cron.Jobs() {
		for _, tag := range job.Tags() {
			jobs = append(jobs, data.CronJobs{Note: tag, ScheduledTime: job.ScheduledTime()})
		}
	}
	return
}
