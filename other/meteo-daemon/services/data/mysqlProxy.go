package data

import (
	"bytes"
	"context"
	"fmt"
	hs "meteo-daemon/services/data/internal"
	"net"
	"sort"
	"time"

	"github.com/tatsushid/go-fastping"
)

func (r *MysqlRepository) AddManualToVpn(ctx context.Context, host ToVpnManual) error {
	host.ID = hs.HashNow32()
	err := r.db.Create(&host).Error
	if err != nil {
		return fmt.Errorf("error insert tovpnManual: %w", err)
	}
	r.logger.Debugf("save tovpnManual: %v", host)
	return nil
}

func (r *MysqlRepository) EditManualToVpn(ctx context.Context, host ToVpnManual) error {
	err := r.db.Save(&host).Error
	if err != nil {
		return fmt.Errorf("error update tovpnManual: %w", err)
	}
	r.logger.Debugf("save tovpnManual: %v", host)
	return nil
}

func (r *MysqlRepository) DelManualToVpn(ctx context.Context, id uint32) error {
	host := ToVpnManual{ID: id}
	err := r.db.Delete(&host).Error
	if err != nil {
		return fmt.Errorf("error delete tovpnManual: %w", err)
	}
	r.logger.Debugf("remove tovpnManual: %v", host)
	return nil
}

func (r *MysqlRepository) GetManualToVpnByName(ctx context.Context, name string) (ToVpnManual, error) {
	var host ToVpnManual
	err := r.db.Where("hostname = ?", name).First(&host).Error
	if err != nil {
		return host, fmt.Errorf("error read tovpnManual: %w", err)
	}
	r.logger.Debugf("read tovpnManual: %v", host)
	return host, nil
}

func (r *MysqlRepository) GetManualToVpn(ctx context.Context, id uint32) (*ToVpnManual, error) {
	var host ToVpnManual
	err := r.db.Where("id = ?", id).First(&host).Error
	if err != nil {
		return nil, fmt.Errorf("error read tovpnManual: %w", err)
	}
	r.logger.Debugf("read tovpnManual: %v", host)
	return &host, nil
}

func (r *MysqlRepository) GetAllAccessLists(ctx context.Context) ([]AccesList, error) {
	var lists []AccesList
	err := r.db.Find(&lists).Error
	if err != nil {
		return nil, fmt.Errorf("error read access_lists: %w", err)
	}
	r.logger.Debugf("read access_lists: %v", lists)
	return lists, nil
}

func (r *MysqlRepository) GetAllManualToVpn(ctx context.Context) ([]ToVpnManual, error) {
	var hosts []ToVpnManual
	err := r.db.Find(&hosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read tovpnManual: %w", err)
	}
	r.logger.Debugf("read tovpnManual: %v", hosts)
	return hosts, nil
}

func (r *MysqlRepository) AddAutoToVpn(ctx context.Context, host ToVpnAuto) error {
	err := r.db.Create(&host).Error
	if err != nil {
		return fmt.Errorf("error insert tovpnAuto: %w", err)
	}
	r.logger.Debugf("save tovpnAuto: %v", host)
	return nil
}

func (r *MysqlRepository) DelAutoToVpn(ctx context.Context, id string) error {
	err := r.db.Where("hostname = ?", id).Delete(&ToVpnAuto{}).Error
	if err != nil {
		return fmt.Errorf("error delete tovpnAuto: %w", err)
	}
	r.logger.Debugf("remove tovpnAuto: %v", id)
	return nil
}

func (r *MysqlRepository) RestoreAutoToVpn(ctx context.Context, id string) error {
	tx := r.db.Begin()
	err := tx.Create(&ToVpnAuto{ID: id}).Error
	if err != nil {
		return fmt.Errorf("error create tovpnAuto: %w", err)
	}
	err = tx.Where("hostname = ?", id).Delete(&ToVpnIgnore{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error delete tovpnIgnore: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("move from tovpnIgnore to tovpnAuto: %v", id)
	return nil

}

func (r *MysqlRepository) GetAutoToVpn(ctx context.Context, id string) (*ToVpnAuto, error) {
	var host ToVpnAuto
	err := r.db.First(&host, id).Error
	if err != nil {
		return nil, fmt.Errorf("error read tovpnAuto: %w", err)
	}
	r.logger.Debugf("read tovpnAuto: %v", host)
	return &host, nil
}

func (r *MysqlRepository) GetAllAutoToVpn(ctx context.Context) ([]ToVpnAuto, error) {
	var hosts []ToVpnAuto
	err := r.db.Order("createdat DESC").Find(&hosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read tovpnAuto: %w", err)
	}
	r.logger.Debugf("read tovpnAuto: %v", hosts)
	return hosts, nil
}

func (r *MysqlRepository) AddIgnoreToVpn(ctx context.Context, id string) error {
	tx := r.db.Begin()
	err := tx.Create(&ToVpnIgnore{ID: id}).Error
	if err != nil {
		return fmt.Errorf("error create tovpnIgnore: %w", err)
	}
	err = tx.Where("hostname = ?", id).Delete(&ToVpnAuto{}).Error
	if err != nil {
		tx.Rollback()
		return fmt.Errorf("error delete tovpnAuto: %w", err)
	}
	tx.Commit()
	r.logger.Debugf("move from tovpnAuto to tovpnIgnore: %v", id)
	return nil
}

func (r *MysqlRepository) DelIgnoreToVpn(ctx context.Context, id string) error {
	err := r.db.Where("hostname = ?", id).Delete(&ToVpnIgnore{}).Error
	if err != nil {
		return fmt.Errorf("error delete tovpnIgnore: %w", err)
	}
	r.logger.Debugf("remove tovpnIgnore: %v", id)
	return nil
}

func (r *MysqlRepository) GetAllIgnore(ctx context.Context) ([]ToVpnIgnore, error) {
	var hosts []ToVpnIgnore
	err := r.db.Find(&hosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read tovpnIgnore: %w", err)
	}
	r.logger.Debugf("read tovpnIgnore: %v", hosts)
	return hosts, nil
}

func (r *MysqlRepository) AddBlockHost(ctx context.Context, host Blocklist) error {
	err := r.db.Create(&host).Error
	if err != nil {
		return fmt.Errorf("error insert blocklist: %w", err)
	}
	r.logger.Debugf("save blocklist: %v", host)
	return nil
}

func (r *MysqlRepository) ClearBlocklist(ctx context.Context) error {
	err := r.db.Delete(&Blocklist{}).Error
	if err != nil {
		return fmt.Errorf("error delete blocklist: %w", err)
	}
	r.logger.Debug("remove all blocklist")
	return nil
}

func (r *MysqlRepository) GetAllBlockHosts(ctx context.Context) ([]Blocklist, error) {
	var hosts []Blocklist
	err := r.db.Find(&hosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read blocklist: %w", err)
	}
	r.logger.Debugf("read blocklist: %v", hosts)
	return hosts, nil
}

func (r *MysqlRepository) AddHomeZoneHost(ctx context.Context, host Homezone) error {
	host.ID = hs.HashString32(fmt.Sprintf("%s%s", host.DomainName, host.IPv4))
	err := r.db.Create(&host).Error
	if err != nil {
		return fmt.Errorf("error insert homezones: %w", err)
	}
	r.logger.Debugf("save homezones: %v", host)
	return nil
}

func (r *MysqlRepository) EditHomeZoneHost(ctx context.Context, host Homezone) error {
	err := r.db.Save(&host).Error
	if err != nil {
		return fmt.Errorf("error update homezones: %w", err)
	}
	r.logger.Debugf("save homezones: %v", host)
	return nil
}

func (r *MysqlRepository) DelHomeZoneHost(ctx context.Context, id uint32) error {
	err := r.db.Delete(&Homezone{ID: id}).Error
	if err != nil {
		return fmt.Errorf("error delete homezones: %w", err)
	}
	r.logger.Debugf("remove homezones: %v", id)
	return nil
}

func (r *MysqlRepository) GetHomeZoneHost(ctx context.Context, id uint32) (*Homezone, error) {
	var host Homezone
	err := r.db.Where("id = ?", id).First(&host).Error
	if err != nil {
		return nil, fmt.Errorf("error read homezones: %w", err)
	}
	r.logger.Debugf("read homezone: %v", host)
	return &host, nil
}

func (r *MysqlRepository) GetAllHomeZoneHosts(ctx context.Context) ([]Homezone, error) {
	var hosts []Homezone
	err := r.db.Order("ip").Find(&hosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read homezones: %w", err)
	}
	sort.Slice(hosts, func(i, j int) bool {
		iIP := net.ParseIP(hosts[i].IPv4)
		jIP := net.ParseIP(hosts[j].IPv4)
		return bytes.Compare(iIP, jIP) < 0
	})
	err = pinger(hosts)
	if err != nil {
		return nil, fmt.Errorf("pinger error: %w", err)
	}
	r.logger.Debugf("read homezones: %v", hosts)
	return hosts, nil
}

func pinger(hosts []Homezone) error {
	ips := map[string]bool{}
	for _, host := range hosts {
		ips[host.IPv4] = false
	}

	p := fastping.NewPinger()
	for k := range ips {
		ra, err := net.ResolveIPAddr("ip4:icmp", k)
		if err != nil {
			return fmt.Errorf("ip parse error: %w", err)
		}
		p.AddIPAddr(ra)
	}
	p.OnRecv = func(addr *net.IPAddr, rtt time.Duration) {
		ips[addr.String()] = true
	}
	p.OnIdle = func() {
		for i, host := range hosts {
			hosts[i].Active = ips[host.IPv4]
		}
	}
	err := p.Run()
	if err != nil {
		return fmt.Errorf("ip ping error: %w", err)
	}
	return nil
}
