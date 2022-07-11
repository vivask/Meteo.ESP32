package mikrotik

import (
	"context"
	"errors"
	"fmt"
	"net"
	"time"

	"github.com/go-git/go-git/v5"
	"github.com/go-git/go-git/v5/plumbing/object"
	"github.com/go-git/go-git/v5/plumbing/transport/http"
	gitssh "github.com/go-git/go-git/v5/plumbing/transport/ssh"
	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

const defaultRemoteName = "origin"
const defaultServiceName = "Mikrotiks"

func (m *Mikrotik) gitPush(repo string) error {
	r, err := git.PlainOpen(repo)
	if err != nil {
		m.logger.Error(err)
		return err
	}

	w, err := r.Worktree()
	if err != nil {
		m.logger.Error(err)
		return err
	}

	status, _ := w.Status()
	if status.IsClean() {
		return nil
	}

	_, err = w.Add(".")
	if err != nil {
		m.logger.Error(err)
		return err
	}

	commit, err := w.Commit("master", &git.CommitOptions{
		Author: &object.Signature{
			Name: "Vivask",
			When: time.Now(),
		},
	})
	if err != nil {
		m.logger.Error(err)
		return err
	}
	_, err = r.CommitObject(commit)
	if err != nil {
		m.logger.Error(err)
		return err
	}

	/*auth, err := getSSHAuth(host)
	if err != nil {
		eLog.Println(err)
		return
	}*/

	auth, err := m.getHTTPAuth(defaultServiceName)
	if err != nil {
		m.logger.Error(err)
		return err
	}

	err = r.Push(&git.PushOptions{RemoteName: defaultRemoteName, Auth: auth})
	if err != nil {
		m.logger.Error(err)
		return err
	}

	//return usedKey(host)
	return m.usedHTTPAccount(defaultServiceName)
}

func (m *Mikrotik) makeKey(host string) (signer ssh.Signer, err error) {
	if row, err := m.repo.GetKeyGitByOwner(context.Background(), host); err != nil {
		return nil, fmt.Errorf("get key from repo: %w", err)
	} else {
		signer, err = ssh.ParsePrivateKey([]byte(row.Finger))
		if err != nil {
			return nil, fmt.Errorf("parse key error: %w", err)
		}
	}
	return
}

func (m *Mikrotik) usedKey(host string) error {
	err := m.repo.UpTimeGitKeys(context.Background(), host)
	if err != nil {
		return fmt.Errorf("update timestamp gitKeys: %w", err)
	}
	return nil
}

func (m *Mikrotik) usedHTTPAccount(service string) error {
	err := m.repo.UpTimeGitUsers(context.Background(), service)
	if err != nil {
		return fmt.Errorf("update timestamp gitUsers: %w", err)
	}
	return nil
}

func (m *Mikrotik) getSSHAuth(host string) (*gitssh.PublicKeys, error) {
	singer, err := m.makeKey(host)
	if err != nil {
		return nil, fmt.Errorf("make key error: %w", err)
	}

	var keyErr *knownhosts.KeyError
	kh, err := m.checkKnownHosts()
	if err != nil {
		return nil, fmt.Errorf("check knownhosts erro:r %w", err)
	}

	auth := &gitssh.PublicKeys{
		Signer: singer,
		/*HostKeyCallbackHelper: gitssh.HostKeyCallbackHelper{HostKeyCallback: ssh.InsecureIgnoreHostKey()},*/
		HostKeyCallbackHelper: gitssh.HostKeyCallbackHelper{
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
		},
	}

	return auth, nil
}

func (m *Mikrotik) getHTTPAuth(service string) (*http.BasicAuth, error) {

	row, err := m.repo.GetUserKeyByService(context.Background(), service)
	if err != nil {
		return nil, fmt.Errorf("Not found account for service: %s, error: %v", service, err)
	}
	auth := &http.BasicAuth{
		Username: row.Username,
		Password: row.Password,
	}
	return auth, nil
}
