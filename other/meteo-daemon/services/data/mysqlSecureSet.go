package data

import (
	"context"
	"fmt"
	hs "meteo-daemon/services/data/internal"
	"time"
)

func (r *MysqlRepository) AddSshKey(ctx context.Context, sshKey SSHKeys) error {
	sshKey.ID = hs.HashNow32()
	err := r.db.Omit("UpdatedAt").Create(&sshKey).Error
	if err != nil {
		return fmt.Errorf("error insert ssh_keys: %w", err)
	}
	r.logger.Debugf("save ssh_keys: %v", sshKey)
	return nil
}

func (r *MysqlRepository) DelSshKey(ctx context.Context, id uint32) error {
	sshKeys := SSHKeys{ID: id}
	err := r.db.Delete(&sshKeys).Error
	if err != nil {
		return fmt.Errorf("error delete ssh_keys: %w", err)
	}
	r.logger.Debugf("remove ssh_keys: %v", sshKeys)
	return nil
}

func (r *MysqlRepository) GetAllSshKeys(ctx context.Context) ([]SSHKeys, error) {
	var keys []SSHKeys
	err := r.db.Order("created desc").Find(&keys).Error
	if err != nil {
		return nil, fmt.Errorf("error read knowhosts: %w", err)
	}
	for i, key := range keys {
		keys[i].ShortFinger = key.Finger[36:52]
		keys[i].HasRecentActivity = !key.UpdatedAt.IsZero()
	}
	r.logger.Debugf("read ssh_keys: %v", keys)
	return keys, err
}

func (r *MysqlRepository) DelKnowhost(ctx context.Context, id uint32) error {
	knownhost := Knowhosts{ID: id}
	err := r.db.Delete(&knownhost).Error
	if err != nil {
		return fmt.Errorf("error delete knownhost: %w", err)
	}
	r.logger.Debugf("remove knownhost: %v", knownhost)
	return nil
}

func (r *MysqlRepository) AddGitKey(ctx context.Context, gitKey GitKeys) error {
	gitKey.ID = hs.HashNow32()
	err := r.db.Omit("UpdatedAt").Create(&gitKey).Error
	if err != nil {
		return fmt.Errorf("error insert git_keys: %w", err)
	}
	r.logger.Debugf("save git_keys: %v", gitKey)
	return nil
}

func (r *MysqlRepository) DelGitKey(ctx context.Context, id uint32) error {
	gitKeys := GitKeys{ID: id}
	err := r.db.Delete(&gitKeys).Error
	if err != nil {
		return fmt.Errorf("error delete git_keys: %w", err)
	}
	r.logger.Debugf("remove git_keys: %v", gitKeys)
	return nil
}

func (r *MysqlRepository) GetAllGitKeys(ctx context.Context) ([]GitKeys, error) {
	var keys []GitKeys
	query := `SELECT *, SUBSTRING(finger, 36, 52) AS short_finger, 
	IF(used IS NOT NULL, 1, 0) AS activity 
	FROM git_keys
	ORDER BY created`
	err := r.db.Raw(query).Scan(&keys).Error
	if err != nil {
		return nil, fmt.Errorf("error read git_keys: %w", err)
	}
	r.logger.Debugf("read git_keys: %v", keys)
	return keys, err
}

func (r *MysqlRepository) AddGitUser(ctx context.Context, gitUser GitUsers) error {
	gitUser.ID = hs.HashNow32()
	err := r.db.Omit("UpdatedAt").Create(&gitUser).Error
	if err != nil {
		return fmt.Errorf("error insert git_users: %w", err)
	}
	r.logger.Debugf("save git_users: %v", gitUser)
	return nil
}

func (r *MysqlRepository) DelGitUser(ctx context.Context, id uint32) error {
	gitUsers := GitUsers{ID: id}
	err := r.db.Delete(&gitUsers).Error
	if err != nil {
		return fmt.Errorf("error delete git_users: %w", err)
	}
	r.logger.Debugf("remove git_users: %v", gitUsers)
	return nil
}

func (r *MysqlRepository) GetAllGitUsers(ctx context.Context) ([]GitUsers, error) {
	var users []GitUsers
	err := r.db.Order("created desc").Find(&users).Error
	if err != nil {
		return nil, fmt.Errorf("error read git_users: %w", err)
	}
	for i, user := range users {
		users[i].HasRecentActivity = !user.UpdatedAt.IsZero()
	}
	r.logger.Debugf("read git_users: %v", users)
	return users, err
}

func (r *MysqlRepository) GetAllKnowhosts(ctx context.Context) ([]Knowhosts, error) {
	var knowhosts []Knowhosts
	err := r.db.Order("created desc").Find(&knowhosts).Error
	if err != nil {
		return nil, fmt.Errorf("error read knowhosts: %w", err)
	}
	for i, host := range knowhosts {
		knowhosts[i].ShortFinger = host.Finger[36:52]
		knowhosts[i].HasRecentActivity = !host.UpdatedAt.IsZero()
	}
	r.logger.Debugf("read knowhosts: %v", knowhosts)
	return knowhosts, err
}

func (r *MysqlRepository) AddKnowhost(ctx context.Context, host Knowhosts) error {
	host.ID = hs.HashString32(host.Host)
	err := r.db.Omit("UpdatedAt").Create(&host).Error
	if err != nil {
		return fmt.Errorf("error insert knowhosts: %w", err)
	}
	r.logger.Debugf("read knowhosts: %v", host)
	return err
}

func (r *MysqlRepository) UpTimeKnowhosts(ctx context.Context, host string) error {
	knowhost := Knowhosts{}
	err := r.db.Where("host = ?", host).First(&knowhost).Error
	if err != nil {
		return fmt.Errorf("read knowhosts error: %w", err)
	}
	knowhost.UpdatedAt = time.Now()
	err = r.db.Save(&knowhost).Error
	if err != nil {
		return fmt.Errorf("update knowhosts error: %w", err)
	}
	r.logger.Debugf("save knowhosts: %v", knowhost)
	return nil
}

func (r *MysqlRepository) GetKeyGitByOwner(ctx context.Context, owner string) (*GitKeys, error) {
	var gitKeys GitKeys
	err := r.db.Where("owner = ?", owner).First(&gitKeys).Error
	if err != nil {
		return nil, fmt.Errorf("error read git_keys: %w", err)
	}
	r.logger.Debugf("read git_keys: %v", gitKeys)
	return &gitKeys, err
}

func (r *MysqlRepository) UpTimeGitKeys(ctx context.Context, owner string) error {
	var gitKey GitKeys
	err := r.db.Model(&gitKey).Where("owner = ?", owner).Update("used", "CURRENT_TIMESTAMP").Error
	if err != nil {
		return fmt.Errorf("update git_keys error: %w", err)
	}
	r.logger.Debugf("save git_keys: %v", gitKey)
	return nil
}

func (r *MysqlRepository) UpTimeGitUsers(ctx context.Context, service string) error {
	var gitUsers GitUsers
	err := r.db.Model(&gitUsers).Where("service = ?", service).Update("used", "CURRENT_TIMESTAMP").Error
	if err != nil {
		return fmt.Errorf("update git_users error: %w", err)
	}
	r.logger.Debugf("save git_users: %v", gitUsers)
	return nil
}

func (r *MysqlRepository) GetUserKeyByService(ctx context.Context, service string) (*GitUsers, error) {
	var gitUsers GitUsers
	err := r.db.Where("service = ?", service).First(&gitUsers).Error
	if err != nil {
		return nil, fmt.Errorf("error read git_users: %w", err)
	}
	r.logger.Debugf("read git_users: %v", gitUsers)
	return &gitUsers, err

}
