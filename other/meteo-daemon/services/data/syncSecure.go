package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) ReplaceSshKeys(ctx context.Context, keys []SSHKeys) error {
	tx := r.db.Begin()
	query := "DELETE FROM ssh_keys"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete ssh_keys error: %w", err)
	}
	for _, key := range keys {
		query = "INSERT INTO ssh_keys (id, finger, owner, created, used) VALUES(?,?,?,?,?)"
		err := tx.Exec(query, key.ID, key.Finger, key.Owner, key.CreatedAt, key.UpdatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert ssh_keys error: %w", err)
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceGitKeys(ctx context.Context, keys []GitKeys) error {
	tx := r.db.Begin()
	query := "DELETE FROM git_keys"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete git_keys error: %w", err)
	}
	for _, key := range keys {
		query = "INSERT INTO git_keys (id, finger, owner, created, used) VALUES(?,?,?,?,?)"
		err := tx.Exec(query, key.ID, key.Finger, key.Owner, key.CreatedAt, key.UpdatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert git_keys error: %w", err)
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceKnownhosts(ctx context.Context, hosts []Knowhosts) error {
	tx := r.db.Begin()
	query := "DELETE FROM knowhosts"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete knowhosts error: %w", err)
	}
	for _, host := range hosts {
		query = "INSERT INTO knowhosts (id, finger, host, created, used) VALUES(?,?,?,?,?)"
		err := tx.Exec(query, host.ID, host.Finger, host.Host, host.CreatedAt, host.UpdatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert knowhosts error: %w", err)
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceGitUsers(ctx context.Context, users []GitUsers) error {
	tx := r.db.Begin()
	query := "DELETE FROM git_users"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete git_users error: %w", err)
	}
	for _, user := range users {
		query = "INSERT INTO git_users (id, username, password, service, created, used) VALUES(?,?,?,?,?,?)"
		err := tx.Exec(query, user.ID, user.Username, user.Password, user.Service, user.CreatedAt, user.UpdatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert git_users error: %w", err)
		}
	}
	tx.Commit()
	return nil
}
