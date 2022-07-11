package data

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
)

const _RADSENS_ = "radsens"

func (r *MysqlRepository) GetAllRadsens(ctx context.Context) ([]Radsens, error) {
	var radsens []Radsens
	err := r.db.Find(&radsens).Error
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err

}

func (r *MysqlRepository) GetNotSyncRadsens(ctx context.Context) ([]Radsens, error) {
	table, err := r.GetSyncTable(ctx, _RADSENS_)
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	var radsens []Radsens
	if table.SyncedAt.IsZero() {
		err = r.db.Find(&radsens).Error
	} else {
		err = r.db.Where("date_time >= ?", table.SyncedAt).Find(&radsens).Error
	}
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err

}

func (r *MysqlRepository) AddSyncRadsens(ctx context.Context, radsens []Radsens) error {
	r.IgnoreAutoSyncTable(_RADSENS_)
	defer r.RestoreAutoSyncTable(_RADSENS_)

	tx := r.db.Begin()
	count := 0
	for _, v := range radsens {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err := r.UpdatedAtSynTable(ctx, _RADSENS_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to radsens", count, r.lead.Other())
	tx.Commit()
	return nil
}

func NotInRadsens(id uint32, set []Radsens) bool {
	for _, v := range set {
		if v.ID == id {
			return false
		}
	}
	return true
}

func (r *MysqlRepository) SyncRadsens(ctx context.Context) error {

	err := r.LockRadsens(true)
	if err != nil {
		return fmt.Errorf("LockRadsens error: %w", err)
	}
	defer r.UnlockRadsens(true)
	r.IgnoreAutoSyncTable(_RADSENS_)
	defer r.RestoreAutoSyncTable(_RADSENS_)

	body, err := r.cli.GetExt("/verify/db/rest/radsens")
	if err != nil {
		return fmt.Errorf("error GET: %w", err)
	}

	var extRadsens []Radsens
	err = json.Unmarshal(body, &extRadsens)
	if err != nil {
		return fmt.Errorf("unmarshal error: %w", err)
	}

	intRadsens, err := r.GetNotSyncRadsens(ctx)
	if err != nil {
		return fmt.Errorf("error read radsens: %w", err)
	}

	// Search not exist external
	var wg sync.WaitGroup
	var newExt []Radsens
	go func(arr *[]Radsens) {
		for _, v := range intRadsens {
			if NotInRadsens(v.ID, extRadsens) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newExt)
	// Search not exist internal
	var newInt []Radsens
	go func(arr *[]Radsens) {
		for _, v := range extRadsens {
			if NotInRadsens(v.ID, intRadsens) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newInt)
	wg.Add(2)
	wg.Wait()

	_, err = r.cli.PostExt("/verify/db/rest/radsens", newExt)
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

	err = r.UpdatedAtSynTable(ctx, _RADSENS_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to radsens", count, r.lead.Other())

	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceRadsens(ctx context.Context, readings []Radsens) error {
	r.IgnoreAutoSyncTable(_RADSENS_)
	defer r.RestoreAutoSyncTable(_RADSENS_)

	tx := r.db.Begin()
	err := tx.Delete(&Radsens{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete radsens error: %w", err)
	}
	for _, v := range readings {
		err = tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert radsens error: %w", err)
		}
	}
	err = r.UpdatedAtSynTable(ctx, _RADSENS_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}
	tx.Commit()
	return nil
}
