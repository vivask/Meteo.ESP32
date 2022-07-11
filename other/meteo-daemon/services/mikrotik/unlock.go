package mikrotik

import (
	"errors"
	"fmt"
	"meteo-daemon/services/data"
	"net"
	"strings"
	s "strings"

	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

func (m *Mikrotik) PutInVpn(adderess, port, username string, host data.ToVpnManual) (err error) {

	var keyErr *knownhosts.KeyError
	kh, err := m.checkKnownHosts()
	if err != nil {
		return fmt.Errorf("check knownhosts erro:r %w", err)
	}

	config := &ssh.ClientConfig{
		User: m.config.VPNUser,
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

	name := host.Name
	if s.HasSuffix(name, ".") {
		name = name[:len(name)-1]
	}
	if len(host.ListID) == 0 {
		host.ListID = m.config.VPNList
	}
	cmd := fmt.Sprintf("/ip firewall address-list add list=%s address=%s", host.ListID, name)
	okReq := adderess + ": $"
	_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}

	return nil
}

func (m *Mikrotik) RemoveFromVpn(adderess, port, username string, host data.ToVpnManual) (err error) {

	var keyErr *knownhosts.KeyError
	kh, err := m.checkKnownHosts()
	if err != nil {
		return fmt.Errorf("check knownhosts erro:r %w", err)
	}

	config := &ssh.ClientConfig{
		User: m.config.VPNUser,
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

	if len(host.ListID) == 0 {
		host.ListID = m.config.VPNList
	}
	cmd := "/ip firewall address-list print"
	req, err := m.execMikrotikNoCheck(adderess, port, username, cmd, config)
	if err != nil {
		return err
	}
	split := strings.Split(req, "\n")
	found := "-1"
	for _, s := range split {
		words := strings.Fields(s)
		if len(words) > 2 && words[1] == host.ListID && words[2] == host.Name {
			//m.logger.Infof("ID: %s, List: %s, Name: %s", words[0], words[1], words[2])
			found = words[0]
		}
	}
	if found == "-1" {
		return fmt.Errorf("not found host %s in access list %s", host.Name, m.config.VPNList)
	}
	cmd = fmt.Sprintf("/ip firewall address-list remove %s", found)
	okReq := adderess + ": $"
	_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}

	return nil

}
