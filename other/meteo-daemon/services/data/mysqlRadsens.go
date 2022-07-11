package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetRadsensPerDay(ctx context.Context, p Period) ([]Radsens, error) {
	var radsens []Radsens
	query := `SELECT static, dynamic, pulse, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN static = (SELECT MAX(static) FROM radsens 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN dynamic = (SELECT MAX(dynamic) FROM radsens  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM radsens  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY hour 
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&radsens).Error
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err
}

func (r *MysqlRepository) GetRadsensPerWeek(ctx context.Context, p Period) ([]Radsens, error) {
	var radsens []Radsens
	query := `SELECT static, dynamic, pulse, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN static = (SELECT MAX(static) FROM radsens 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN dynamic = (SELECT MAX(dynamic) FROM radsens  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM radsens  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&radsens).Error
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err
}

func (r *MysqlRepository) GetRadsensPerMonth(ctx context.Context, p Period) ([]Radsens, error) {
	var radsens []Radsens
	query := `SELECT static, dynamic, pulse, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN static = (SELECT MAX(static) FROM radsens 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN dynamic = (SELECT MAX(dynamic) FROM radsens  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM radsens  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&radsens).Error
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err
}

func (r *MysqlRepository) GetRadsensPerYear(ctx context.Context, p Period) ([]Radsens, error) {
	var radsens []Radsens
	query := `SELECT static, dynamic, pulse, date_time, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN static = (SELECT MAX(static) FROM radsens 
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN dynamic = (SELECT MAX(dynamic) FROM radsens  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM radsens  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&radsens).Error
	if err != nil {
		return nil, fmt.Errorf("error read radsens: %w", err)
	}
	r.logger.Debugf("read radsens: %v", radsens)
	return radsens, err
}
