package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetLoging(ctx context.Context, p Period) ([]Logging, error) {
	var loging []Logging
	query := `SELECT id, message, type, date_time, 
	DATE_FORMAT(date_time,'%H:%i:%s') as time,
	DATE_FORMAT(date_time, '%Y-%m-%d') as date
	FROM logging 
	WHERE date_time >= ?  
	AND date_time <= ?
	ORDER BY date_time DESC`
	err := r.db.Raw(query, p.Begin, p.End).Scan(&loging).Error
	if err != nil {
		return nil, fmt.Errorf("error read loging: %w", err)
	}
	r.logger.Debugf("read loging: %v", loging)
	return loging, err
}

func (r *MysqlRepository) JournalClear(ctx context.Context) error {
	err := r.db.Exec("DELETE FROM logging").Error
	if err != nil {
		return fmt.Errorf("error delete loging: %w", err)
	}
	var settings Settings
	err = r.db.Model(&settings).Updates(Settings{ClearJournalEsp32: true}).Error
	if err != nil {
		return fmt.Errorf("update settings error: %w", err)
	}
	return nil
}
