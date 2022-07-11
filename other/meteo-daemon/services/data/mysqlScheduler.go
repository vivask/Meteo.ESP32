package data

import (
	"context"
	"fmt"
	hs "meteo-daemon/services/data/internal"
	"meteo-daemon/utils"
	"time"
)

func (r *MysqlRepository) AddJob(ctx context.Context, job Jobs) error {
	if job.Period.ID == "one" {
		dt, err := utils.GetDateTime(job.Date, job.Time)
		if err != nil {
			return fmt.Errorf("GetDateTime error: %w", err)
		}
		if dt.Unix()-time.Now().Unix() < 0 {
			return fmt.Errorf("task can't be run at this time: %v", job.Time)
		}
	}
	// check implementation task
	url := fmt.Sprintf("/verify/schedule/rest/task/implemented/%v", job.Task.ID)
	_, err := r.cli.GetInt(url)
	if err != nil {
		return fmt.Errorf("Task has not implementation: %w", err)
	}
	tx := r.db.Begin()
	job.ID = hs.HashNow32()
	for _, param := range job.Params {
		param.ID = hs.HashNow32()
		param.JobID = job.ID
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error create job_params: %w", err)
		}
	}
	err = tx.Create(&job).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error create job: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("create job: %v", job)
	// try execute job
	url = fmt.Sprintf("/verify/schedule/rest/job/create/%v", job.ID)
	_, err = r.cli.PutInt(url, nil)
	if err != nil {
		err := r.db.Model(&Jobs{}).Where("id = ?", job.ID).Update("active", 0).Error
		if err != nil {
			return fmt.Errorf("update jobs error: %w", err)
		}
		return fmt.Errorf("Can't run job: %d", job.ID)
	}
	return nil
}

func (r *MysqlRepository) EditJob(ctx context.Context, job Jobs) error {
	if job.Period.ID == "one" {
		dt, err := utils.GetDateTime(job.Date, job.Time)
		if err != nil {
			return fmt.Errorf("GetDateTime error: %w", err)
		}
		if dt.Unix()-time.Now().Unix() < 0 {
			return fmt.Errorf("task can't be run at this time: %v", job.Time)
		}

	}
	// check implementation task
	url := fmt.Sprintf("/verify/schedule/rest/task/implemented/%v", job.Task.ID)
	_, err := r.cli.GetInt(url)
	if err != nil {
		return fmt.Errorf("Task has not implementation: %w", err)
	}
	tx := r.db.Begin()
	err = tx.Where("job_id = ?", job.ID).Delete(&JobParams{}).Error
	if err != nil {
		return fmt.Errorf("remove job_params error: %w", err)
	}
	for _, param := range job.Params {
		param.ID = hs.HashNow32()
		param.JobID = job.ID
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error create job_params: %w", err)
		}
	}
	err = tx.Save(&job).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error update jobs: %w", err)
	}
	r.logger.Debugf("save jobs: %v", job)
	tx.Commit()
	// try execute job
	url = fmt.Sprintf("/verify/schedule/rest/job/update/%v", job.ID)
	_, err = r.cli.PutInt(url, nil)
	if err != nil {
		err := r.db.Model(&Jobs{}).Where("id = ?", job.ID).Update("active", 0).Error
		if err != nil {
			return fmt.Errorf("update jobs error: %w", err)
		}
		return fmt.Errorf("Can't run job: %d", job.ID)
	}
	return nil
}

func (r *MysqlRepository) ActivateJob(ctx context.Context, id uint32) error {
	job, err := r.GetJob(ctx, id)
	if err != nil {
		return fmt.Errorf("error GetJob: %w", err)
	}
	if job.Period.ID == "one" {
		dt, err := utils.GetDateTime(job.Date, job.Time)
		if err != nil {
			return fmt.Errorf("GetDateTime error: %w", err)
		}
		if dt.Unix()-time.Now().Unix() < 0 {
			return fmt.Errorf("task can't be run at this time: %v", job.Time)
		}
	}
	active := 0
	if job.Active == 0 {
		active = 1
	}
	err = r.db.Model(&Jobs{}).Where("id = ?", id).Update("active", active).Error
	if err != nil {
		return fmt.Errorf("update jobs error: %w", err)
	}
	// try execute job
	url := fmt.Sprintf("/verify/schedule/rest/job/update/%v", id)
	_, err = r.cli.PutInt(url, nil)
	if err != nil {
		err := r.db.Model(&Jobs{}).Where("id = ?", id).Update("active", 0).Error
		if err != nil {
			return fmt.Errorf("update jobs error: %w", err)
		}
		return fmt.Errorf("Can't run job: %d", id)
	}
	_, err = r.cli.PutExt("/verify/schedule/rest/job/reload", nil)
	if err != nil {
		return fmt.Errorf("remote reload jobs fail: %w", err)
	}
	return nil
}

func (r *MysqlRepository) DeactivateJob(ctx context.Context, id uint32, off bool) error {
	err := r.db.Model(&Jobs{}).Where("id = ?", id).Update("active", 0).Error
	if err != nil {
		return fmt.Errorf("update jobs error: %w", err)
	}
	if off {
		url := fmt.Sprintf("/verify/schedule/rest/job/update/%v", id)
		_, err = r.cli.PutInt(url, nil)
		if err != nil {
			return fmt.Errorf("Can't execute job: %d", id)
		}
		_, err = r.cli.PutExt("/verify/schedule/rest/job/reload", nil)
		if err != nil {
			return fmt.Errorf("remote reload jobs fail: %w", err)
		}
	}
	return nil
}

func (r *MysqlRepository) RunJob(ctx context.Context, id uint32) error {
	url := fmt.Sprintf("/verify/schedule/rest/job/run/%v", id)
	_, err := r.cli.PutInt(url, nil)
	if err != nil {
		return fmt.Errorf("RunJob error: %w", err)
	}
	return nil
}

func (r *MysqlRepository) DelJob(ctx context.Context, id uint32) error {
	err := r.db.Delete(&Jobs{ID: id}).Error
	if err != nil {
		return fmt.Errorf("error delete jobs: %w", err)
	}
	url := fmt.Sprintf("/verify/schedule/rest/job/remove/%v", id)
	_, err = r.cli.DeleteInt(url)
	if err != nil {
		return fmt.Errorf("DelJob error: %w", err)
	}
	r.logger.Debugf("remove jobs: %v", id)
	return nil
}

func (r *MysqlRepository) GetJob(ctx context.Context, id uint32) (*Jobs, error) {
	job := Jobs{ID: id}
	err := r.db.Preload("Period").Preload("Task").Preload("Day").First(&job).Error
	if err != nil {
		return nil, fmt.Errorf("error read jobs: %w", err)
	}
	job.Params, err = r.GetJobParams(ctx, job.ID)
	if err != nil {
		return nil, fmt.Errorf("error read job_params: %w", err)
	}
	r.logger.Debugf("read jobs: %v", job)
	return &job, err
}

func (r *MysqlRepository) GetAllJobs(ctx context.Context) ([]Jobs, error) {
	var jobs []Jobs
	err := r.db.Order("note asc").Preload("Period").Preload("Task").Preload("Day").Find(&jobs).Error
	if err != nil {
		return nil, fmt.Errorf("error read jobs: %w", err)
	}
	for i, job := range jobs {
		jobs[i].Params, err = r.GetJobParams(ctx, job.ID)
		if err != nil {
			return nil, fmt.Errorf("error read job_params: %w", err)
		}
	}
	r.logger.Debugf("read jobs: %v", jobs)
	return jobs, err
}

func (r *MysqlRepository) GetAllActiveJobs(ctx context.Context) ([]Jobs, error) {
	var jobs []Jobs
	err := r.db.Order("note asc").Where("active = 1").Preload("Period").Preload("Task").Preload("Day").Find(&jobs).Error
	if err != nil {
		return nil, fmt.Errorf("error read jobs: %w", err)
	}
	for i, job := range jobs {
		jobs[i].Params, err = r.GetJobParams(ctx, job.ID)
		if err != nil {
			return nil, fmt.Errorf("error read job_params: %w", err)
		}
	}
	r.logger.Debugf("read jobs: %v", jobs)
	return jobs, err
}

func (r *MysqlRepository) GetJobParams(ctx context.Context, id uint32) ([]JobParams, error) {
	var params []JobParams
	err := r.db.Where("job_id = ?", id).Order("name desc").Find(&params).Error
	if err != nil {
		return nil, fmt.Errorf("error read job_params: %w", err)
	}
	r.logger.Debugf("read [job_id: %d] job_params: %v", id, params)
	return params, err
}

func (r *MysqlRepository) GetJobParamsById(ctx context.Context, id uint32) ([]JobParams, error) {
	var params []JobParams
	err := r.db.Where("job_id = ?", id).Order("id desc").Find(&params).Error
	if err != nil {
		return nil, fmt.Errorf("error read job_params: %w", err)
	}
	r.logger.Debugf("read job_params: %v", params)
	return params, err
}

func (r *MysqlRepository) GetAllJobParams(ctx context.Context) ([]JobParams, error) {
	var params []JobParams
	err := r.db.Order("name desc").Find(&params).Error
	if err != nil {
		return nil, fmt.Errorf("error read job_params: %w", err)
	}
	r.logger.Debugf("read job_params: %v", params)
	return params, nil
}

func (r *MysqlRepository) AddTask(ctx context.Context, task Tasks) error {
	tx := r.db.Begin()
	for _, param := range task.Params {
		param.ID = hs.HashNow32()
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error create task_params: %w", err)
		}
	}
	err := tx.Create(&task).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error create tasks: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("create tasks: %v", task)
	return nil
}

func (r *MysqlRepository) EditTask(ctx context.Context, task Tasks) error {
	tx := r.db.Begin()
	err := tx.Where("task_id = ?", task.ID).Delete(&TaskParams{}).Error
	if err != nil {
		return fmt.Errorf("error delete task_params: %w", err)
	}
	for _, param := range task.Params {
		param.ID = hs.HashNow32()
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error create task_params: %w", err)
		}
	}
	err = tx.Save(&task).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error update tasks: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("save task: %v", task)

	return nil
}

func (r *MysqlRepository) DelTask(ctx context.Context, id string) error {
	err := r.db.Delete(&Tasks{ID: id}).Error
	if err != nil {
		return fmt.Errorf("error delete tasks: %w", err)
	}
	r.logger.Debugf("remove tasks: %v", id)
	return nil
}

func (r *MysqlRepository) GetTask(ctx context.Context, id string) (*Tasks, error) {
	task := Tasks{ID: id}
	err := r.db.First(&task).Error
	if err != nil {
		return nil, fmt.Errorf("error read tasks: %w", err)
	}
	task.Params, err = r.GetTaskParams(ctx, id)
	if err != nil {
		return nil, fmt.Errorf("error read task_params: %w", err)
	}
	r.logger.Debugf("read tasks: %v", task)
	return &task, nil
}

func (r *MysqlRepository) GetAllTasks(ctx context.Context) ([]Tasks, error) {
	var tasks []Tasks
	err := r.db.Order("name desc").Find(&tasks).Error
	if err != nil {
		return nil, fmt.Errorf("error read tasks: %w", err)
	}
	for i, task := range tasks {
		tasks[i].Params, err = r.GetTaskParams(ctx, task.ID)
		if err != nil {
			return nil, fmt.Errorf("error read task_params: %w", err)
		}
	}
	r.logger.Debugf("read tasks: %v", tasks)
	return tasks, nil
}

func (r *MysqlRepository) GetTaskParams(ctx context.Context, task_id string) ([]TaskParams, error) {
	var params []TaskParams
	err := r.db.Where("task_id = ?", task_id).Find(&params).Error
	if err != nil {
		return params, fmt.Errorf("error read task_params: %w", err)
	}
	r.logger.Debugf("read task_params: %v", params)
	return params, nil
}

func (r *MysqlRepository) GetAllPeriods(ctx context.Context) ([]Periods, error) {
	var periods []Periods
	err := r.db.Order("idx asc").Find(&periods).Error
	if err != nil {
		return nil, fmt.Errorf("error read periods: %w", err)
	}
	r.logger.Debugf("read periods: %v", periods)
	return periods, nil
}

func (r *MysqlRepository) GetAllDays(ctx context.Context) ([]Days, error) {
	var days []Days
	err := r.db.Find(&days).Error
	if err != nil {
		return nil, fmt.Errorf("error read days: %w", err)
	}
	r.logger.Debugf("read days: %v", days)
	return days, nil
}

func (r *MysqlRepository) GetAllExecutors(ctx context.Context) ([]Executors, error) {
	var executors []Executors
	err := r.db.Find(&executors).Error
	if err != nil {
		return nil, fmt.Errorf("error read executors: %w", err)
	}
	r.logger.Debugf("read days: %v", executors)
	return executors, nil
}
