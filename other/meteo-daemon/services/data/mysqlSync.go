package data

import (
	"context"
	"fmt"
	hs "meteo-daemon/services/data/internal"
	"time"
)

func (r *MysqlRepository) LockWriteTable(name string) error {
	query := fmt.Sprintf("TABLES LOCK %s WRITE", name)
	err := r.db.Raw(query).Error
	if err != nil {
		return fmt.Errorf("can't lock table [%s], error: %w", name, err)
	}
	return nil
}

func (r *MysqlRepository) UnlockTables() {
	err := r.db.Raw("TABLES UNLOCK").Error
	if err != nil {
		r.logger.Errorf("can't unlock tables, error: %v", err)
	}
}

func (r *MysqlRepository) AddSyncTable(ctx context.Context, table SyncTables) error {
	tx := r.db.Begin()
	for _, param := range table.Params {
		param.ID = hs.HashString32(fmt.Sprintf("%s%s", param.SyncTableID, param.SyncType))
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error create sync_params: %w", err)
		}
	}
	err := tx.Create(&table).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error create sync_tables: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("create sync_tables: %v", table)

	return nil
}

func (r *MysqlRepository) EditSyncTable(ctx context.Context, table SyncTables, id string) error {
	tx := r.db.Begin()
	err := tx.Delete(&SyncParams{}, "table_id = ?", id).Error
	if err != nil {
		return fmt.Errorf("error delete sync_params: %w", err)
	}
	for _, param := range table.Params {
		param.ID = hs.HashString32(fmt.Sprintf("%s%s", param.SyncTableID, param.SyncType))
		err := tx.Create(&param).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("error update sync_params: %w", err)
		}
	}
	err = tx.Model(&table).Where("name = ?", id).Updates(table).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error update sync_tables: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("save sync_tables: %v", table)

	return nil
}

func (r *MysqlRepository) DelSyncTable(ctx context.Context, id string) error {
	table := SyncTables{ID: id}
	err := r.db.Where("name = ?", id).Delete(&table).Error
	if err != nil {
		return fmt.Errorf("error delete sync_tables: %w", err)
	}
	r.logger.Debugf("remove sync_tables: %v", table)
	return nil
}

func (r *MysqlRepository) GetSyncTable(ctx context.Context, id string) (*SyncTables, error) {
	table := SyncTables{}
	err := r.db.Where("name = ?", id).First(&table).Error
	if err != nil {
		return nil, fmt.Errorf("error read sync_tables: %w", err)
	}
	table.Params, err = r.GetSyncTableParams(ctx, id)
	if err != nil {
		return nil, fmt.Errorf("error read sync_params: %w", err)
	}
	r.logger.Debugf("read sync_tables: %v", table)
	return &table, nil
}

func (r *MysqlRepository) GetAllSyncTables(ctx context.Context) ([]SyncTables, error) {
	var tables []SyncTables
	err := r.db.Order("name asc").Find(&tables).Error
	if err != nil {
		return nil, fmt.Errorf("error read tables: %w", err)
	}
	for i, table := range tables {
		tables[i].Params, err = r.GetSyncTableParams(ctx, table.ID)
		if err != nil {
			return nil, fmt.Errorf("error read job_params: %w", err)
		}
	}
	r.logger.Debugf("read tables: %v", tables)
	return tables, nil
}

func (r *MysqlRepository) GetSyncTableParams(ctx context.Context, table_id string) ([]SyncParams, error) {
	var params []SyncParams
	err := r.db.Where("table_id = ?", table_id).Find(&params).Error
	if err != nil {
		return params, fmt.Errorf("error read sync_params: %w", err)
	}
	r.logger.Debugf("read sync_params: %v", params)
	return params, nil
}

func (r *MysqlRepository) ReplaceExtBmx280(ctx context.Context) error {
	data, err := r.GetAllBmx280(ctx)
	if err != nil {
		return fmt.Errorf("GetAllBmx280 error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/bmx280", data)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtDs18b20(ctx context.Context) error {
	data, err := r.GetAllDs18b20(ctx)
	if err != nil {
		return fmt.Errorf("GetAllDs18b20 error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/ds18b20", data)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtZe08ch2o(ctx context.Context) error {
	data, err := r.GetAllZe08ch2o(ctx)
	if err != nil {
		return fmt.Errorf("GetAllZe08ch2o error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/ze08ch2o", data)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtRadsens(ctx context.Context) error {
	data, err := r.GetAllRadsens(ctx)
	if err != nil {
		return fmt.Errorf("GetAllZe08ch2o error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/radsens", data)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtHomeZones(ctx context.Context) error {
	err := r.LockWriteTable("homezones")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllHomeZoneHosts(ctx)
	if err != nil {
		return fmt.Errorf("GetAllHomeZoneHosts error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/homezones", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtToVpnManual(ctx context.Context) error {
	err := r.LockWriteTable("tovpn_manuals")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllManualToVpn(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllManualToVpn error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/tovpnmanual", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtToVpnAuto(ctx context.Context) error {
	err := r.LockWriteTable("tovpn_autos")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllAutoToVpn(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllAutoToVpn error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/tovpnauto", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtToVpnIgnore(ctx context.Context) error {
	err := r.LockWriteTable("tovpn_ignores")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllIgnore(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllIgnore error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/tovpnignore", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtSshKeys(ctx context.Context) error {
	err := r.LockWriteTable("ssh_keys")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllSshKeys(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllSshKeys error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/sshkeys", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtGitKeys(ctx context.Context) error {
	err := r.LockWriteTable("git_keys")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllGitKeys(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllGitKeys error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/gitkeys", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtGitUsers(ctx context.Context) error {
	err := r.LockWriteTable("git_users")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	hosts, err := r.GetAllGitUsers(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllGitUsers error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/gitusers", hosts)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtTasks(ctx context.Context) error {
	err := r.LockWriteTable("tasks")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	tasks, err := r.GetAllTasks(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllTasks error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/tasks", tasks)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) ReplaceExtJobs(ctx context.Context) error {
	err := r.LockWriteTable("jobs")
	if err != nil {
		return fmt.Errorf("lock table error: %w", err)
	}
	defer r.UnlockTables()
	jobs, err := r.GetAllJobs(context.Background())
	if err != nil {
		return fmt.Errorf("GetAllJobs error: %w", err)
	}
	_, err = r.cli.PostExt("/verify/db/rest/replace/jobs", jobs)
	if err != nil {
		return err
	}
	return nil
}

func (r *MysqlRepository) UpdatedAtSynTable(ctx context.Context, name string) error {
	table, err := r.GetSyncTable(ctx, name)
	if err != nil {
		return fmt.Errorf("GetSyncTable error: %w", err)
	}
	table.SyncedAt = time.Now()
	err = r.db.Model(table).Where("name = ?", name).Updates(table).Error
	if err != nil {
		return fmt.Errorf("update sync_tables error: %w", err)
	}
	return nil
}
