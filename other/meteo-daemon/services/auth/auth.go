package auth

import (
	"crypto/hmac"
	"crypto/sha256"
	"encoding/hex"
	"errors"
	"fmt"
	"io/ioutil"
	"meteo-daemon/domain"
	"meteo-daemon/services/data"
	"meteo-daemon/utils"
	"os"
	"time"

	"github.com/dgrijalva/jwt-go"
	"github.com/sirupsen/logrus"
	"golang.org/x/crypto/bcrypt"
)

// Authentication interface lists the methods that our authentication service should implement
type Authentication interface {
	Authenticate(reqUser *data.User, user *data.User) bool
	GenerateAccessToken(user *data.User) (string, error)
	GenerateRefreshToken(user *data.User) (string, error)
	GenerateCustomKey(userID string, password string) string
	ValidateAccessToken(token string) (string, error)
	ValidateRefreshToken(token string) (string, string, error)
	Logger() *logrus.Entry
	Config() *Config
}

// RefreshTokenCustomClaims specifies the claims for refresh token
type RefreshTokenCustomClaims struct {
	UserID    string
	CustomKey string
	KeyType   string
	jwt.StandardClaims
}

// AccessTokenCustomClaims specifies the claims for access token
type AccessTokenCustomClaims struct {
	UserID  string
	KeyType string
	jwt.StandardClaims
}

// AuthService is the implementation of our Authentication
type AuthService struct {
	hasp    domain.StartStopInterface
	logger  *logrus.Entry
	logfile *os.File
	config  *Config
}

func New(ss domain.StartStopInterface, lg *logrus.Entry, cnf *Config) *AuthService {
	return &AuthService{
		hasp:    ss,
		logger:  lg,
		logfile: nil,
		config:  cnf,
	}
}

// Authenticate checks the user credentials in request against the db and authenticates the request
func (auth *AuthService) Authenticate(reqUser *data.User, user *data.User) bool {

	if err := bcrypt.CompareHashAndPassword([]byte(user.Password), []byte(reqUser.Password)); err != nil {
		auth.logger.Debug("password hashes are not same")
		return false
	}
	return true
}

// GenerateRefreshToken generate a new refresh token for the given user
func (auth *AuthService) GenerateRefreshToken(user *data.User) (string, error) {

	cusKey := auth.GenerateCustomKey(user.ID, user.Token)
	tokenType := "refresh"

	claims := RefreshTokenCustomClaims{
		user.ID,
		cusKey,
		tokenType,
		jwt.StandardClaims{
			Issuer: "bookite.auth.service",
		},
	}

	signBytes, err := ioutil.ReadFile(auth.config.RefreshTokenPrivateKeyPath)
	if err != nil {
		auth.logger.Error("unable to read private key ", "error: ", err)
		return "", errors.New("could not generate refresh token. please try again later")
	}

	signKey, err := jwt.ParseRSAPrivateKeyFromPEM(signBytes)
	if err != nil {
		auth.logger.Error("unable to parse private key ", "error: ", err)
		return "", errors.New("could not generate refresh token. please try again later")
	}

	token := jwt.NewWithClaims(jwt.SigningMethodRS256, claims)

	return token.SignedString(signKey)
}

// GenerateAccessToken generates a new access token for the given user
func (auth *AuthService) GenerateAccessToken(user *data.User) (string, error) {

	userID := user.ID
	tokenType := "access"

	claims := AccessTokenCustomClaims{
		userID,
		tokenType,
		jwt.StandardClaims{
			ExpiresAt: time.Now().Add(time.Minute * time.Duration(auth.config.JwtExpiration)).Unix(),
			Issuer:    "bookite.auth.service",
		},
	}

	signBytes, err := ioutil.ReadFile(auth.config.AccessTokenPrivateKeyPath)
	if err != nil {
		auth.logger.Error("unable to read private key ", "error: ", err)
		return "", errors.New("could not generate access token. please try again later")
	}

	signKey, err := jwt.ParseRSAPrivateKeyFromPEM(signBytes)
	if err != nil {
		auth.logger.Error("unable to parse private key ", "error: ", err)
		return "", errors.New("could not generate access token. please try again later")
	}

	token := jwt.NewWithClaims(jwt.SigningMethodRS256, claims)

	return token.SignedString(signKey)
}

// GenerateCustomKey creates a new key for our jwt payload
// the key is a hashed combination of the userID and user tokenhash
func (auth *AuthService) GenerateCustomKey(userID string, tokenHash string) string {

	// data := userID + tokenHash
	h := hmac.New(sha256.New, []byte(tokenHash))
	h.Write([]byte(userID))
	sha := hex.EncodeToString(h.Sum(nil))
	return sha
}

// ValidateAccessToken parses and validates the given access token
// returns the userId present in the token payload
func (auth *AuthService) ValidateAccessToken(tokenString string) (string, error) {

	token, err := jwt.ParseWithClaims(tokenString, &AccessTokenCustomClaims{}, func(token *jwt.Token) (interface{}, error) {
		if _, ok := token.Method.(*jwt.SigningMethodRSA); !ok {
			auth.logger.Error("Unexpected signing method in auth token")
			return nil, errors.New("Unexpected signing method in auth token")
		}
		verifyBytes, err := ioutil.ReadFile(auth.config.AccessTokenPublicKeyPath)
		if err != nil {
			auth.logger.Error("unable to read public key", "error", err)
			return nil, err
		}

		verifyKey, err := jwt.ParseRSAPublicKeyFromPEM(verifyBytes)
		if err != nil {
			auth.logger.Error("unable to parse public key ", "error: ", err)
			return nil, err
		}

		return verifyKey, nil
	})

	if err != nil {
		auth.logger.Error("unable to parse claims ", "error: ", err)
		return "", err
	}

	claims, ok := token.Claims.(*AccessTokenCustomClaims)
	if !ok || !token.Valid || claims.UserID == "" || claims.KeyType != "access" {
		return "", errors.New("invalid token: authentication failed")
	}
	return claims.UserID, nil
}

// ValidateRefreshToken parses and validates the given refresh token
// returns the userId and customkey present in the token payload
func (auth *AuthService) ValidateRefreshToken(tokenString string) (string, string, error) {

	token, err := jwt.ParseWithClaims(tokenString, &RefreshTokenCustomClaims{}, func(token *jwt.Token) (interface{}, error) {
		if _, ok := token.Method.(*jwt.SigningMethodRSA); !ok {
			auth.logger.Error("Unexpected signing method in auth token")
			return nil, errors.New("Unexpected signing method in auth token")
		}
		verifyBytes, err := ioutil.ReadFile(auth.config.RefreshTokenPublicKeyPath)
		if err != nil {
			auth.logger.Error("unable to read public key ", "error: ", err)
			return nil, err
		}

		verifyKey, err := jwt.ParseRSAPublicKeyFromPEM(verifyBytes)
		if err != nil {
			auth.logger.Error("unable to parse public key ", "error: ", err)
			return nil, err
		}

		return verifyKey, nil
	})

	if err != nil {
		auth.logger.Error("unable to parse claims ", "error: ", err)
		return "", "", err
	}

	claims, ok := token.Claims.(*RefreshTokenCustomClaims)
	auth.logger.Debug("ok", ok)
	if !ok || !token.Valid || claims.UserID == "" || claims.KeyType != "refresh" {
		auth.logger.Debug("could not extract claims from token")
		return "", "", errors.New("invalid token: authentication failed")
	}
	return claims.UserID, claims.CustomKey, nil
}

func (auth *AuthService) Logger() *logrus.Entry {
	return auth.logger
}

func (auth *AuthService) Config() *Config {
	return auth.config
}

func (AuthService) StartNum() int {
	return 6
}

func (auth *AuthService) Enabled() bool {
	return auth.config.Active
}

func (auth *AuthService) IsRun() bool {
	return auth.hasp.IsRun()
}

func (auth *AuthService) Start() error {
	if !auth.hasp.Start() {
		return fmt.Errorf("%s:failed to start", auth.config.Title)
	}

	auth.logger.Debugf("%s: success started", auth.config.Title)

	return nil
}

func (auth *AuthService) Stop() error {
	if !auth.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", auth.config.Title)
	}

	auth.logger.Debugf("%s: success stoped", auth.config.Title)

	if auth.logfile != nil {
		auth.logfile.Close()
	}

	return nil
}

func (auth *AuthService) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, auth.config.LogFile, auth.config.LogLevel, file, auth.logger)
	if err != nil {
		return err
	}
	auth.logfile = f
	return nil
}

type Config struct {
	Title                      string `toml:"Title"`
	Active                     bool   `toml:"Active"`
	AccessTokenPrivateKeyPath  string `toml:"AccessTokenPrivateKeyPath"`
	AccessTokenPublicKeyPath   string `toml:"AccessTokenPublicKeyPath"`
	RefreshTokenPrivateKeyPath string `toml:"RefreshTokenPrivateKeyPath"`
	RefreshTokenPublicKeyPath  string `toml:"RefreshTokenPublicKeyPath"`
	MailVerifCodeExpiration    int    `toml:"MailVerifCodeExpiration"` // in hours
	PassResetCodeExpiration    int    `toml:"PassResetCodeExpiration"` // in minutes
	JwtExpiration              int    `toml:"JwtExpiration"`           // in minutes
	JwtKey                     string `toml:"JwtKey"`
	LogFile                    string `toml:"LogFile"`
	LogLevel                   string `toml:"LogLevel"`
}
