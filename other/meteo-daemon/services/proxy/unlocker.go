package proxy

import (
	"context"
	"fmt"
	"meteo-daemon/client"
	"meteo-daemon/services/data"
	"strings"

	"github.com/sirupsen/logrus"
)

type Unlocker struct {
	logger   *logrus.Entry
	repo     data.Repository
	cli      *client.Client
	unlocked map[string]struct{}
	ignored  map[string]struct{}
}

func NewUnlocker(lg *logrus.Entry, r data.Repository, c *client.Client) *Unlocker {

	return &Unlocker{
		logger:   lg,
		repo:     r,
		cli:      c,
		unlocked: make(map[string]struct{}),
		ignored:  make(map[string]struct{}),
	}
}

func (un *Unlocker) LoadHosts() (unlocked int, ignorered int) {
	hosts, err := un.repo.GetAllAutoToVpn(context.Background())
	if err != nil {
		un.logger.Error(err)
		return
	}
	for _, host := range hosts {
		name := host.ID
		if !strings.HasSuffix(name, ".") {
			name += "."
		}
		un.unlocked[name] = struct{}{}
		unlocked++
	}
	ignor, err := un.repo.GetAllIgnore(context.Background())
	if err != nil {
		un.logger.Error(err)
		return
	}
	for _, host := range ignor {
		name := host.ID
		if !strings.HasSuffix(name, ".") {
			name += "."
		}
		un.ignored[name] = struct{}{}
		ignorered++
	}
	return
}

func (un *Unlocker) Exist(name string) bool {
	_, exist := un.unlocked[name]
	return exist
}

func (un *Unlocker) Ignore(name string) bool {
	_, exist := un.ignored[name]
	return exist
}

func (un *Unlocker) AddIgnore(name string) bool {
	if un.Exist(name) && !un.Ignore(name) {
		un.ignored[name] = struct{}{}
		return true
	}
	return false
}

func (un *Unlocker) AddHost(name string) bool {
	if !un.Exist(name) && !un.Ignore(name) {
		un.unlocked[name] = struct{}{}
		return true
	}
	return false
}

func (un *Unlocker) RemoveHost(name string) {
	if un.Exist(name) {
		delete(un.unlocked, name)
	}
}

func (un *Unlocker) RemoveIgnore(name string) {
	if un.Ignore(name) {
		delete(un.ignored, name)
	}
}

func LoadUnlocker(lg *logrus.Entry, repo data.Repository, c *client.Client) *Unlocker {

	list := NewUnlocker(lg, repo, c)
	unlocked, ignored := list.LoadHosts()
	lg.Info("Loaded ", unlocked, " unlocked hosts from database, ignore: ", ignored)
	return list
}

func (un *Unlocker) InsertManual(host data.ToVpnManual) error {
	if !un.AddHost(host.Name) {
		return fmt.Errorf("Host [%s] exist, can't insert", host.Name)
	}
	err := un.repo.AddManualToVpn(context.Background(), host)
	if err != nil {
		return fmt.Errorf("Can't insert records to tovpn_manuals: %w", err)
	}
	_, err = un.cli.PutInt("/verify/mikrotiks/rest/tovpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}
	return nil
}

func (un *Unlocker) UpdateManual(host data.ToVpnManual) error {
	err := un.RemoveManual(host.ID)
	if err != nil {
		return fmt.Errorf("RemoveManual error: %w", err)
	}

	err = un.InsertManual(host)
	if err != nil {
		return fmt.Errorf("InsertManual error: %w", err)
	}
	return nil
}

func (un *Unlocker) RemoveManual(id uint32) error {
	host, err := un.repo.GetManualToVpn(context.Background(), id)
	if err != nil {
		return fmt.Errorf("GetManualToVpn error: %w", err)
	}
	un.RemoveHost(host.Name)

	err = un.repo.DelManualToVpn(context.Background(), host.ID)
	if err != nil {
		return fmt.Errorf("DelManualToVpn error: %w", err)
	}

	_, err = un.cli.PutInt("/verify/mikrotiks/rest/rmvpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}
	return nil
}

func (un *Unlocker) InsertHost(name string) error {
	if !un.AddHost(name) {
		return fmt.Errorf("Host [%s] exist, can't insert", name)
	}
	err := un.repo.AddAutoToVpn(context.Background(), data.ToVpnAuto{ID: name})
	if err != nil {
		return fmt.Errorf("Can't insert records to tovpn_autos: %w", err)
	}
	host := data.ToVpnManual{Name: name}
	_, err = un.cli.PutInt("/verify/mikrotiks/rest/tovpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}
	return nil
}

func (un *Unlocker) DeleteHost(id string) error {
	err := un.repo.DelAutoToVpn(context.Background(), id)
	if err != nil {
		return fmt.Errorf("DelAutoToVpn error: %w", err)
	}
	un.RemoveHost(id)
	host := data.ToVpnManual{Name: id}
	_, err = un.cli.PutInt("/verify/mikrotiks/rest/rmvpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}
	return nil
}

func (un *Unlocker) IgnoreHost(name string) error {
	err := un.repo.AddIgnoreToVpn(context.Background(), name)
	if err != nil {
		return fmt.Errorf("AddIgnoreToVpn error: %w", err)
	}
	un.AddIgnore(name)

	err = un.repo.DelAutoToVpn(context.Background(), name)
	if err != nil {
		return fmt.Errorf("DelAutoToVpn error: %w", err)
	}
	un.RemoveHost(name)
	host := data.ToVpnManual{Name: name}
	_, err = un.cli.PutInt("/verify/mikrotiks/rest/rmvpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}

	return nil
}

func (un *Unlocker) DeleteIgnore(host string) error {
	err := un.repo.DelIgnoreToVpn(context.Background(), host)
	if err != nil {
		return fmt.Errorf("DelIgnoreToVpn error: %w", err)
	}
	un.RemoveIgnore(host)
	return nil
}

func (un *Unlocker) RestoreHost(name string) error {
	err := un.repo.RestoreAutoToVpn(context.Background(), name)
	if err != nil {
		return fmt.Errorf("Can't insert records to tovpn_autos: %w", err)
	}
	un.AddHost(name)

	host := data.ToVpnManual{Name: name}
	_, err = un.cli.PutInt("/verify/mikrotiks/rest/tovpn", host)
	if err != nil {
		un.logger.Errorf("error GET mikrotiks: %v", err)
	}

	err = un.repo.DelIgnoreToVpn(context.Background(), name)
	if err != nil {
		return fmt.Errorf("DelIgnoreToVpn error: %w", err)
	}
	un.RemoveIgnore(name)

	return nil
}
