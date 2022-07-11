package data

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
)

const _BMX280_ = "bmx280"

func (r *MysqlRepository) GetAllBmx280(ctx context.Context) ([]Bmx280, error) {
	var bmx280 []Bmx280
	err := r.db.Find(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err

}

func (r *MysqlRepository) GetNotSyncBmx280(ctx context.Context) ([]Bmx280, error) {
	table, err := r.GetSyncTable(ctx, _BMX280_)
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	var bmx280 []Bmx280
	if table.SyncedAt.IsZero() {
		err = r.db.Find(&bmx280).Error
	} else {
		err = r.db.Where("date_time >= ?", table.SyncedAt).Find(&bmx280).Error
	}
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err

}

func (r *MysqlRepository) AddSyncBmx280(ctx context.Context, bmx280 []Bmx280) error {
	r.IgnoreAutoSyncTable(_BMX280_)
	defer r.RestoreAutoSyncTable(_BMX280_)

	tx := r.db.Begin()
	count := 0
	for _, v := range bmx280 {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err := r.UpdatedAtSynTable(ctx, _BMX280_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}
	r.logger.Infof("Received and insert [%d] records from %s to bmx280", count, r.lead.Other())
	tx.Commit()
	return nil
}

func NotInBmx280(id uint32, set []Bmx280) bool {
	for _, v := range set {
		if v.ID == id {
			return false
		}
	}
	return true
}

func (r *MysqlRepository) SyncBmx280(ctx context.Context) error {

	err := r.LockBmx280(true)
	if err != nil {
		return fmt.Errorf("LockBmx280 error: %w", err)
	}
	defer r.UnlockBmx280(true)
	r.IgnoreAutoSyncTable(_BMX280_)
	defer r.RestoreAutoSyncTable(_BMX280_)

	body, err := r.cli.GetExt("/verify/db/rest/bmx280")
	if err != nil {
		return fmt.Errorf("error GET: %w", err)
	}

	var extBmx280 []Bmx280
	err = json.Unmarshal(body, &extBmx280)
	if err != nil {
		return fmt.Errorf("unmarshal error: %w", err)
	}

	intBmx280, err := r.GetNotSyncBmx280(ctx)
	if err != nil {
		return fmt.Errorf("error read bmx280: %w", err)
	}

	// Search not exist external
	var wg sync.WaitGroup
	var newExt []Bmx280
	go func(arr *[]Bmx280) {
		for _, v := range intBmx280 {
			if NotInBmx280(v.ID, extBmx280) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newExt)
	var newInt []Bmx280
	go func(arr *[]Bmx280) {
		for _, v := range extBmx280 {
			if NotInBmx280(v.ID, intBmx280) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newInt)
	wg.Add(2)
	wg.Wait()

	_, err = r.cli.PostExt("/verify/db/rest/bmx280", newExt)
	if err != nil {
		return fmt.Errorf("error POST: %w", err)
	}

	tx := r.db.Begin()
	// Search not exist internal and insert
	count := 0
	for _, v := range newInt {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err = r.UpdatedAtSynTable(ctx, _BMX280_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to bmx280", count, r.lead.Other())

	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceBmx280(ctx context.Context, readings []Bmx280) error {
	r.IgnoreAutoSyncTable(_BMX280_)
	defer r.RestoreAutoSyncTable(_BMX280_)

	tx := r.db.Begin()
	err := tx.Delete(&Bmx280{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete bmx280 error: %w", err)
	}
	for _, v := range readings {
		err = tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert bmx280 error: %w", err)
		}
	}
	err = r.UpdatedAtSynTable(ctx, _BMX280_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}
	tx.Commit()
	return nil
}
