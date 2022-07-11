package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) GetBmx280PerDayAvg(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN press = (SELECT AVG(press) FROM bmx280 
	WHERE (date_time >= (? - ?)))
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT AVG(hum) FROM bmx280  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM AVG' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY hour 
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerDayMin(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN press = (SELECT MIN(press) FROM bmx280 
	WHERE (date_time >= (? - ?)))
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MIN(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MIN(hum) FROM bmx280  
	WHERE (date_time >= (? - ?)))  
	THEN 'I AM AVG' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ? 
	GROUP BY hour 
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerDayMax(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%H:00:00') as hour, 
	CASE WHEN press = (SELECT MAX(press) FROM bmx280 
	WHERE (date_time >= (? - ?)))
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MAX(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?)))
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MAX(hum) FROM bmx280  
	WHERE (date_time >= (? - ?)))
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY hour 
	ORDER BY date_time DESC 
	LIMIT 24`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerWeekAvg(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT AVG(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT AVG(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerWeekMin(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT MIN(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MIN(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MIN(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerWeekMax(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT MAX(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MAX(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MAX(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 7`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerMonthAvg(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT AVG(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT AVG(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerMonthMin(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT MIN(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MIN(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MIN(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerMonthMax(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%w') as day, 
	DATE_FORMAT(date_time, '%Y-%m-%d') as dateonly, 
	CASE WHEN press = (SELECT MAX(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MAX(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MAX(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY day 
	ORDER BY date_time DESC 
	LIMIT 30`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerYearAvg(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN press = (SELECT AVG(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT AVG(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT AVG(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM AVG' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerYearMin(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN press = (SELECT MIN(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MIN(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MIN(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MIN' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}

func (r *MysqlRepository) GetBmx280PerYearMax(ctx context.Context, p Period) ([]Bmx280, error) {
	var bmx280 []Bmx280
	query := `SELECT press, tempr, hum, date_time, 
	DATE_FORMAT(date_time, '%c') as month, 
	DATE_FORMAT(date_time, '%M') as monthonly, 
	CASE WHEN press = (SELECT MAX(press) FROM bmx280 
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN tempr = (SELECT MAX(tempr) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value, 
	CASE WHEN hum = (SELECT MAX(hum) FROM bmx280  
	WHERE (date_time >= (? - ?))) 
	THEN 'I AM MAX' ELSE NULL  
	END null_value 
	FROM bmx280  
	WHERE date_time >= ? AND date_time <= ?
	GROUP BY month 
	ORDER BY date_time DESC 
	LIMIT 12`
	err := r.db.Raw(query, p.End, p.Begin, p.End, p.Begin, p.End, p.Begin, p.Begin, p.End).Scan(&bmx280).Error
	if err != nil {
		return nil, fmt.Errorf("error read bmx280: %w", err)
	}
	r.logger.Debugf("read bmx280: %v", bmx280)
	return bmx280, err
}
