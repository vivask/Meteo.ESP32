package mikrotik

import (
	"errors"
	"fmt"
	"io/ioutil"
	"net"
	"os"

	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

func (m *Mikrotik) SyncDNS(adderess, port, username string, hosts map[string]string) error {

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

	cmd := "/system script run remove-all-static-dns"
	okReq := adderess + ": $"
	_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}
	fName, err := prepareFile(hosts)
	if err != nil {
		return err
	}
	err = m.sftpUpload("static-dns", fName, adderess, port, config)
	os.Remove(fName)
	if err != nil {
		return fmt.Errorf("Error file upload on: %s, error: %v", adderess, err)
	}

	cmd = "/import file-name=static-dns"
	okReq = "Script file loaded and executed successfully"
	_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}

	cmd = "/file remove static-dns"
	okReq = adderess + ": $"
	_, err = m.execMikrotik(adderess, port, username, cmd, okReq, config)
	if err != nil {
		return err
	}

	return nil
}

func prepareFile(hosts map[string]string) (fName string, err error) {

	file, err := ioutil.TempFile("/tmp", "zones")
	defer file.Close()
	for host, ip := range hosts {
		str := "/ip dns static add name=" + host + " address=" + ip + "\n"
		_, err := file.WriteString(str)
		if err != nil {
			os.Remove(file.Name())
			return "", err
		}
	}
	return file.Name(), err
}
