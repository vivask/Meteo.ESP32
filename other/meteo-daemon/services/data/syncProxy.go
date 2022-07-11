package data

import (
	"context"
	"fmt"
)

func (r *MysqlRepository) ReplaceHomezones(ctx context.Context, hosts []Homezone) error {
	tx := r.db.Begin()
	query := "DELETE FROM homezones"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete homezones error: %w", err)
	}
	for _, host := range hosts {
		query = "INSERT INTO homezones (id, domain_name, ip, mac, note) VALUES(?,?,?,?,?)"
		err := tx.Exec(query, host.ID, host.DomainName, host.IPv4, host.Mac, host.Note).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert homezones error: %w", err)
		}
	}
	tx.Commit()
	_, err = r.cli.PutInt("/verify/proxy/rest/sync", nil)
	if err != nil {
		return fmt.Errorf("reload homezone error: %w", err)
	}
	return nil
}

func (r *MysqlRepository) ReplaceToVpnManual(ctx context.Context, hosts []ToVpnManual) error {
	tx := r.db.Begin()
	query := "DELETE FROM tovpn_manuals"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete tovpn_manuals error: %w", err)
	}
	for _, host := range hosts {
		query = "INSERT INTO tovpn_manuals (id, hostname, note) VALUES(?,?,?)"
		err := tx.Exec(query, host.ID, host.Name, host.Note).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert tovpn_manuals error: %w", err)
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceToVpnAuto(ctx context.Context, hosts []ToVpnAuto) error {
	tx := r.db.Begin()
	query := "DELETE FROM tovpn_autos"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete tovpn_autos error: %w", err)
	}
	for _, host := range hosts {
		query = "INSERT INTO tovpn_autos (hostname, createdat) VALUES(?,?)"
		err := tx.Exec(query, host.ID, host.CreatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert tovpn_autos error: %w", err)
		}
	}
	tx.Commit()
	return nil
}

func (r *MysqlRepository) ReplaceToVpnIgnore(ctx context.Context, hosts []ToVpnIgnore) error {
	tx := r.db.Begin()
	query := "DELETE FROM tovpn_ignores"
	err := tx.Exec(query).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("delete tovpn_ignores error: %w", err)
	}
	for _, host := range hosts {
		query = "INSERT INTO tovpn_ignores (hostname, updatedat) VALUES(?,?)"
		err := tx.Exec(query, host.ID, host.UpdatedAt).Error
		if err != nil {
			tx.Rollback()
			return fmt.Errorf("insert tovpn_ignores error: %w", err)
		}
	}
	tx.Commit()
	return nil
}
