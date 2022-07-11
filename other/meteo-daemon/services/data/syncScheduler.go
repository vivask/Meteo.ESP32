package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) ReplaceTasks(ctx context.Context, tasks []Tasks) error {
	tx := r.db.Begin()
	query := "DELETE FROM task_params"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete task_params error: %w", err)
	}
	query = "DELETE FROM tasks"
	err = tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete tasks error: %w", err)
	}

	for _, task := range tasks {
		query = "INSERT INTO tasks (id, name, note) VALUES(?,?,?)"
		err := tx.Exec(query, task.ID, task.Name, task.Note).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert tasks error: %w", err)
		}
		for _, param := range task.Params {
			query = "INSERT INTO task_params (id, name, task_id) VALUES(?,?,?)"
			err := tx.Exec(query, param.ID, param.Name, param.TaskID).Error
			if err != nil {
				tx.Rollback()
				return fmt.Errorf("insert task_params error: %w", err)
			}
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceJobs(ctx context.Context, jobs []Jobs) error {
	tx := r.db.Begin()
	query := "DELETE FROM job_params"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete job_params error: %w", err)
	}
	query = "DELETE FROM jobs"
	err = tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete jobs error: %w", err)
	}

	for _, job := range jobs {
		query = `INSERT INTO jobs (id, note, active, value, time, date, verbose, 
			executor_id, task_id, period_id, day_id) VALUES(?,?,?,?,?,?,?,?,?,?,?)`
		err := tx.Exec(query, job.ID, job.Note, job.Active, job.Value, job.Time,
			job.Date, job.Verbose, job.Executor, job.TaskID, job.PeriodID, job.DayID).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert jobs error: %w", err)
		}
		for _, param := range job.Params {
			query = "INSERT INTO job_params (id, name, value, job_id) VALUES(?,?,?,?)"
			err := tx.Exec(query, param.ID, param.Name, param.Value, param.JobID).Error
			if err != nil {
				tx.Rollback()
				return fmt.Errorf("insert job_params error: %w", err)
			}
		}
	}
	tx.Commit()
	return nil
}
