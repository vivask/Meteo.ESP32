package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetDs18b20PerDay(ctx context.Context, p Period) ([]Ds18b20, error) {
	var ds18b20 []Ds18b20
	query := `SELECT tempr, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM ds18b20 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ds18b20 
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY hour   
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ds18b20).Error
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}

func (r *MysqlRepository) GetDs18b20PerWeek(ctx context.Context, p Period) ([]Ds18b20, error) {
	var ds18b20 []Ds18b20
	query := `SELECT tempr, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM ds18b20 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ds18b20 
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ds18b20).Error
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}

func (r *MysqlRepository) GetDs18b20PerMonth(ctx context.Context, p Period) ([]Ds18b20, error) {
	var ds18b20 []Ds18b20
	query := `SELECT tempr, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM ds18b20 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ds18b20 
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ds18b20).Error
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}

func (r *MysqlRepository) GetDs18b20PerYear(ctx context.Context, p Period) ([]Ds18b20, error) {
	var ds18b20 []Ds18b20
	query := `SELECT tempr, date_time, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM ds18b20 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM ds18b20 
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.Begin, p.End).Scan(&ds18b20).Error
	if err != nil {
		return nil, fmt.Errorf("error read ds18b20: %w", err)
	}
	r.logger.Debugf("read ds18b20: %v", ds18b20)
	return ds18b20, err
}
