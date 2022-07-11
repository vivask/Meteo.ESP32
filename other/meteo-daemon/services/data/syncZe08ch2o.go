package data

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
)

const _ZE08CH2O_ = "ze08ch2o"

func (r *MysqlRepository) GetAllZe08ch2o(ctx context.Context) ([]Ze08ch2o, error) {
	var ze08ch2o []Ze08ch2o
	err := r.db.Find(&ze08ch2o).Error
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}

func (r *MysqlRepository) GetNotSyncZe08ch2o(ctx context.Context) ([]Ze08ch2o, error) {
	table, err := r.GetSyncTable(ctx, _ZE08CH2O_)
	if err != nil {
		return nil, fmt.Errorf("error read tasks: %w", err)
	}
	var ze08ch2o []Ze08ch2o
	if table.SyncedAt.IsZero() {
		err = r.db.Find(&ze08ch2o).Error
	} else {
		err = r.db.Where("date_time >= ?", table.SyncedAt).Find(&ze08ch2o).Error
	}
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}

func (r *MysqlRepository) AddSyncZe08ch2o(ctx context.Context, ze08ch2o []Ze08ch2o) error {
	r.IgnoreAutoSyncTable(_ZE08CH2O_)
	defer r.RestoreAutoSyncTable(_ZE08CH2O_)

	tx := r.db.Begin()
	count := 0
	for _, v := range ze08ch2o {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err := r.UpdatedAtSynTable(ctx, _ZE08CH2O_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to ze08ch2o", count, r.lead.Other())
	tx.Commit()
	return nil
}

func NotInZe08ch2o(id uint32, set []Ze08ch2o) bool {
	for _, v := range set {
		if v.ID == id {
			return false
		}
	}
	return true
}

func (r *MysqlRepository) SyncZe08ch2o(ctx context.Context) error {

	err := r.LockZe08ch2o(true)
	if err != nil {
		return fmt.Errorf("LockZe08ch2o error: %w", err)
	}
	defer r.UnlockZe08ch2o(true)
	r.IgnoreAutoSyncTable(_ZE08CH2O_)
	defer r.RestoreAutoSyncTable(_ZE08CH2O_)

	body, err := r.cli.GetExt("/verify/db/rest/ze08ch2o")
	if err != nil {
		return fmt.Errorf("error GET: %w", err)
	}

	var extZe08ch2o []Ze08ch2o
	err = json.Unmarshal(body, &extZe08ch2o)
	if err != nil {
		return fmt.Errorf("unmarshal error: %w", err)
	}

	intZe08ch2o, err := r.GetNotSyncZe08ch2o(ctx)
	if err != nil {
		return fmt.Errorf("error read ze08ch2o: %w", err)
	}

	// Search not exist external
	var wg sync.WaitGroup
	var newExt []Ze08ch2o
	go func(arr *[]Ze08ch2o) {
		for _, v := range intZe08ch2o {
			if NotInZe08ch2o(v.ID, extZe08ch2o) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newExt)
	// Search not exist internal
	var newInt []Ze08ch2o
	go func(arr *[]Ze08ch2o) {
		for _, v := range extZe08ch2o {
			if NotInZe08ch2o(v.ID, intZe08ch2o) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newInt)
	wg.Add(2)
	wg.Wait()

	_, err = r.cli.PostExt("/verify/db/rest/ze08ch2o", newExt)
	if err != nil {
		return fmt.Errorf("error POST: %w", err)
	}

	tx := r.db.Begin()
	count := 0
	for _, v := range newInt {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err = r.UpdatedAtSynTable(ctx, _ZE08CH2O_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to ze08ch2o", count, r.lead.Other())

	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceZe08ch2o(ctx context.Context, readings []Ze08ch2o) error {
	r.IgnoreAutoSyncTable(_ZE08CH2O_)
	defer r.RestoreAutoSyncTable(_ZE08CH2O_)

	tx := r.db.Begin()
	err := tx.Delete(&Ze08ch2o{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete ze08ch2o error: %w", err)
	}

	for _, v := range readings {
		err = tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert ze08ch2o error: %w", err)
		}
	}

	err = r.UpdatedAtSynTable(ctx, _ZE08CH2O_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	tx.Commit()
	return nil
}
