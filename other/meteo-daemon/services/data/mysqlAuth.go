package data

import (
	"context"
	"fmt"
	"time"
)

// Create inserts the given user into the database
func (r *MysqlRepository) Create(ctx context.Context, user *User) error {
	//user.ID = uuid.NewV4().String()
	user.CreatedAt = time.Now()
	user.UpdatedAt = time.Now()

	r.logger.Info("creating user: %#v", user)
	r.logger.Debugf("creating user with id: %v", user.ID)
	//query := "INSERT INTO USERS (id, username, password, tokenhash, createdat, updatedat) VALUES (?, ?, ?, ?, ?, ?)"
	//err := r.db.Exec(query, user.ID, user.Username, user.Password, user.Token, user.CreatedAt, user.UpdatedAt).Error
	err := r.db.Create(user).Error
	return err
}

// GetUserByID retrieves the user object having the given ID, else returns error
func (r *MysqlRepository) GetUserByID(ctx context.Context, userID string) (*User, error) {
	var user User
	err := r.db.Where("id = ?", userID).First(&user).Error
	if err != nil {
		return nil, fmt.Errorf("error read users id %s, error: %w", userID, err)
	}
	r.logger.Debugf("read user: %v", user)
	return &user, nil
}

// UpdateUsername updates the username of the given user
func (r *MysqlRepository) UpdateUsername(ctx context.Context, user *User) error {
	user.UpdatedAt = time.Now()

	/*query := "UPDATE USERS SET username = ?, updatedat = ? WHERE id = ?"
	if err := r.db.Exec(query, user.Username, user.UpdatedAt, user.ID).Error; err != nil {
		return err
	}*/
	err := r.db.Where("id = ?", user.ID).Update(user).Error
	return err
}

// UpdatePassword updates the user password
func (r *MysqlRepository) UpdatePassword(ctx context.Context, userID string, password string, tokenHash string) error {

	//query := "UPDATE USERS SET password = ?, tokenhash = ? where id = ?"
	//err := r.db.Exec(query, password, tokenHash, userID).Error
	var user User
	err := r.db.Where("id = ?", user.ID).First(&user).Error
	if err != nil {
		return err
	}
	user.Password = password
	user.Token = tokenHash
	err = r.db.Where("id = ?", userID).Update(&user).Error
	return err
}
