package data

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
)

const _DS18B20_ = "ds18b20"

func (r *MysqlRepository) GetAllDs18b20(ctx context.Context) ([]Ds18b20, error) {
	var ds18b20 []Ds18b20
	err := r.db.Find(&ds18b20).Error
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}

func (r *MysqlRepository) GetNotSyncDs18b20(ctx context.Context) ([]Ds18b20, error) {
	table, err := r.GetSyncTable(ctx, _DS18B20_)
	if err != nil {
		return nil, fmt.Errorf("error read tasks: %w", err)
	}
	var ds18b20 []Ds18b20
	if table.SyncedAt.IsZero() {
		err = r.db.Find(&ds18b20).Error
	} else {
		err = r.db.Where("date_time >= ?", table.SyncedAt).Find(&ds18b20).Error
	}
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}

func (r *MysqlRepository) AddSyncDs18b20(ctx context.Context, ds18b20 []Ds18b20) error {
	r.IgnoreAutoSyncTable(_DS18B20_)
	defer r.RestoreAutoSyncTable(_DS18B20_)

	tx := r.db.Begin()
	count := 0
	for _, v := range ds18b20 {
		err := tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert error: %w", err)
		}
		count++
	}

	err := r.UpdatedAtSynTable(ctx, _DS18B20_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}

	r.logger.Infof("Received and insert [%d] records from %s to ds18b20", count, r.lead.Other())

	tx.Commit()
	return nil
}

func NotInDs18b20(id uint32, set []Ds18b20) bool {
	for _, v := range set {
		if v.ID == id {
			return false
		}
	}
	return true
}

func (r *MysqlRepository) SyncDs18b20(ctx context.Context) error {

	err := r.LockDs18b20(true)
	if err != nil {
		return fmt.Errorf("LockZe08ch2o error: %w", err)
	}
	defer r.UnlockDs18b20(true)
	r.IgnoreAutoSyncTable(_DS18B20_)
	defer r.RestoreAutoSyncTable(_DS18B20_)

	body, err := r.cli.GetExt("/verify/db/rest/ds18b20")
	if err != nil {
		return fmt.Errorf("error GET: %w", err)
	}

	var extDs18b20 []Ds18b20
	err = json.Unmarshal(body, &extDs18b20)
	if err != nil {
		return fmt.Errorf("unmarshal error: %w", err)
	}

	intDs18b20, err := r.GetNotSyncDs18b20(ctx)
	if err != nil {
		return fmt.Errorf("error read ds18b20: %w", err)
	}

	// Search not exist external and send
	var wg sync.WaitGroup
	var newExt []Ds18b20
	go func(arr *[]Ds18b20) {
		for _, v := range intDs18b20 {
			if NotInDs18b20(v.ID, extDs18b20) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newExt)
	var newInt []Ds18b20
	go func(arr *[]Ds18b20) {
		for _, v := range extDs18b20 {
			if NotInDs18b20(v.ID, intDs18b20) {
				*arr = append(*arr, v)
			}
		}
		wg.Done()
	}(&newInt)
	wg.Add(2)
	wg.Wait()

	_, err = r.cli.PostExt("/verify/db/rest/ds18b20", newExt)
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

	err = r.UpdatedAtSynTable(ctx, _DS18B20_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}
	r.logger.Infof("Received and insert [%d] records from %s to ds18b20", count, r.lead.Other())

	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceDs18b20(ctx context.Context, readings []Ds18b20) error {
	r.IgnoreAutoSyncTable(_DS18B20_)
	defer r.RestoreAutoSyncTable(_DS18B20_)

	tx := r.db.Begin()
	err := tx.Delete(&Ds18b20{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete ds18b20 error: %w", err)
	}
	for _, v := range readings {
		err = tx.Create(&v).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert ds18b20 error: %w", err)
		}
	}
	err = r.UpdatedAtSynTable(ctx, _DS18B20_)
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("UpdatedAtSynTable error: %w", err)
	}
	tx.Commit()
	return nil
}
