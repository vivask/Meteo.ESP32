package mikrotik

import (
	"errors"
	"fmt"
	"net"
	"regexp"
	"strconv"
	s "strings"
	"time"

	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

const (
	MAX_TIME = 6
	MIN_TIME = 1
)

func (m *Mikrotik) CheckPPPState(adderess, port, username string) error {

	var keyErr *knownhosts.KeyError
	kh, err := m.checkKnownHosts()
	if err != nil {
		return fmt.Errorf("check knownhosts erro:r %w", err)
	}

	config := &ssh.ClientConfig{
		User: username,
		Auth: m.makeKeyring(),
		HostKeyCallback: ssh.HostKeyCallback(func(host string, remote net.Addr, pubKey ssh.PublicKey) error {
			hErr := kh(host, remote, pubKey)
			if errors.As(hErr, &keyErr) && len(keyErr.Want) > 0 {
				m.logger.Infof("WARNING: %v is not a key of %s, either a MiTM attack or %s has reconfigured the host pub key.", hostKeyString(pubKey), host, host)
				return keyErr
			} else if errors.As(hErr, &keyErr) && len(keyErr.Want) == 0 {
				m.logger.Infof("WARNING: %s is not trusted, adding this key: %q to known_hosts file.", host, hostKeyString(pubKey))
				return m.addHostKey(host, remote, pubKey)
			}
			return nil
		}),
	}

	cmd := "/interface pppoe-client monitor " + m.config.PPPIface + " once"
	okReq := "status: connected"
	req, err := m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}
	m.logger.Debugf("PPPOE status: %s", req)

	expr, err := extractExpr(req, "uptime: (.*)")
	if err != nil {
		return err
	}
	ticks, err := getTicks(expr)
	if err != nil {
		return err
	}
	hr := time.Now().Local().Add(ticks).Hour()

	if hr > MAX_TIME || hr < MIN_TIME {
		cmd = "/system scheduler print from " + m.config.PPPScript + " value-list"
		okReq := "name: " + m.config.PPPScript
		req, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
		if err != nil {
			return err
		}
		m.logger.Debugf("PPPScript: %s", req)
		expr, err = extractExpr(req, "start-date: (.*)")
		if err != nil {
			return err
		}
		start_date, err := mikrotikDateToDate(expr)
		if err != nil {
			return err
		}
		date_restart := dateRestart(hr, ticks)
		days := diffDate(date_restart, start_date)
		if days > 1 {
			cmd = "/log warning \"PPP restart scrip activated!\""
			okReq := adderess + ": $"
			_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
			m.logger.Debugf("log warning: %s", req)
			if err != nil {
				return err
			}
			cmd = "/system scheduler set " + m.config.PPPScript + " start-date=" + dateToMikrotikDate(date_restart)
			m.logger.Debugf("scheduler set: %s", req)
			m.logger.Infof("Date restart: %v", dateToMikrotikDate(date_restart))
			_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
			if err != nil {
				return err
			}
		}
	}
	return nil
}

func getTicks(str string) (ticks time.Duration, err error) {

	var duration time.Duration
	var days int
	s := s.Split(str, "d")

	if len(s) > 1 {
		duration, err = time.ParseDuration(s[1])
		if err != nil {
			return 0, err
		}
		days, err = strconv.Atoi(s[0])
	} else {
		duration, err = time.ParseDuration(s[0])
		if err != nil {
			return 0, err
		}
		days = 0
	}

	return time.Duration(432000-duration.Seconds()-float64(days*24*60*60)) * time.Second, nil
}

func extractExpr(str, expr string) (res string, err error) {
	r, err := regexp.Compile(expr)
	if err != nil {
		return
	}
	updatestr := r.FindString(str)
	split := s.Split(updatestr, " ")[1]
	res = split[:len(split)-1]
	return
}

func mikrotikDateToDate(d string) (date time.Time, err error) {

	split := s.Split(d, "/")
	m := split[0]
	day := split[1]
	year := split[2]
	months := map[string]string{"jan": "01", "feb": "02", "mar": "03", "apr": "04", "may": "05", "jun": "06", "jul": "07", "aug": "08", "sep": "09", "oct": "10", "now": "11", "dec": "12"}
	month := months[m]
	date, err = time.Parse("2006-01-02", year+"-"+month+"-"+day)
	return
}

func dateToMikrotikDate(d time.Time) (date string) {

	months := map[int]string{1: "Jan", 2: "Feb", 3: "Mar", 4: "Apr", 5: "May", 6: "Jun", 7: "Jul", 8: "Aug", 9: "Sep", 10: "Oct", 11: "Now", 12: "Dec"}
	return months[int(d.Month())] + "/" + strconv.Itoa(d.Day()) + "/" + strconv.Itoa(d.Year())
}

func diffDate(d1 time.Time, d2 time.Time) (diff int64) {
	u1 := d1.Unix()
	u2 := d2.Unix()
	if u1 > u2 {
		diff = (u1 - u2) / 86400
	} else {
		diff = (u2 - u1) / 86400
	}
	return
}

func dateRestart(hour int, ticks time.Duration) (dr time.Time) {
	if hour >= MIN_TIME && hour <= MAX_TIME {
		dr = time.Now().Local().Add(ticks)
	} else {
		dr = time.Now().Local().AddDate(0, 0, 1)
	}
	return
}
