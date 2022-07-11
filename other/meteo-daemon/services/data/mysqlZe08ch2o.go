package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetZe08ch2oPerDay(ctx context.Context, p Period) ([]Ze08ch2o, error) {
	var ze08ch2o []Ze08ch2o
	query := `SELECT ch2o, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN ch2o = (SELECT MAX(ch2o) FROM ze08ch2o 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ze08ch2o  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY hour 
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ze08ch2o).Error
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}

func (r *MysqlRepository) GetZe08ch2oPerWeek(ctx context.Context, p Period) ([]Ze08ch2o, error) {
	var ze08ch2o []Ze08ch2o
	query := `SELECT date_time, ch2o, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN ch2o = (SELECT MAX(ch2o) FROM ze08ch2o 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ze08ch2o  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ze08ch2o).Error
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}

func (r *MysqlRepository) GetZe08ch2oPerMonth(ctx context.Context, p Period) ([]Ze08ch2o, error) {
	var ze08ch2o []Ze08ch2o
	query := `SELECT date_time, ch2o, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN ch2o = (SELECT MAX(ch2o) FROM ze08ch2o 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ze08ch2o  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ze08ch2o).Error
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}

func (r *MysqlRepository) GetZe08ch2oPerYear(ctx context.Context, p Period) ([]Ze08ch2o, error) {
	var ze08ch2o []Ze08ch2o
	query := `SELECT date_time, ch2o, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN ch2o = (SELECT MAX(ch2o) FROM ze08ch2o 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ze08ch2o  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ze08ch2o).Error
	if err != nil {
		return nil, fmt.Errorf("error read ze08ch2o: %w", err)
	}
	r.logger.Debugf("read ze08ch2o: %v", ze08ch2o)
	return ze08ch2o, err
}
