package auth

import (
	"context"
	"meteo-daemon/services/data"
	"meteo-daemon/utils"
	"net/http"
	"strings"

	"golang.org/x/crypto/bcrypt"
)

// Signup handles signup request
func (ah *AuthHandler) Signup(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	user := r.Context().Value(UserKey{}).(data.User)

	hashedPass, err := ah.hashPassword(user.Password)
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		ToJSON(&GenericResponse{Status: false, Message: UserCreationFailed}, w)
		return
	}
	user.Password = hashedPass
	user.Token = utils.GenerateRandomString(15)

	err = ah.repo.Create(context.Background(), &user)
	if err != nil {
		ah.logger.Error("unable to insert user to database", "error", err)
		errMsg := err.Error()
		if strings.Contains(errMsg, PgDuplicateKeyMsg) {
			w.WriteHeader(http.StatusBadRequest)
			// ToJSON(&GenericError{Error: ErrUserAlreadyExists}, w)
			ToJSON(&GenericResponse{Status: false, Message: ErrUserAlreadyExists}, w)
		} else {
			w.WriteHeader(http.StatusInternalServerError)
			// ToJSON(&GenericError{Error: errMsg}, w)
			ToJSON(&GenericResponse{Status: false, Message: UserCreationFailed}, w)
		}
		return
	}

	ah.logger.Debug("User created successfully")
	w.WriteHeader(http.StatusCreated)
	ToJSON(&GenericResponse{Status: true, Message: "user created successfully"}, w)
}

func (ah *AuthHandler) hashPassword(password string) (string, error) {

	hashedPass, err := bcrypt.GenerateFromPassword([]byte(password), bcrypt.DefaultCost)
	if err != nil {
		ah.logger.Error("unable to hash password ", "error: ", err)
		return "", err
	}

	return string(hashedPass), nil
}

// Login handles login request
func (ah *AuthHandler) Login(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	reqUser := r.Context().Value(UserKey{}).(data.User)

	//ah.logger.Infof("reqUser: %v", reqUser)

	user, err := ah.repo.GetUserByID(context.Background(), reqUser.ID)
	if err != nil {
		ah.logger.Error("error fetching the user", "error", err)
		errMsg := err.Error()
		if strings.Contains(errMsg, PgNoRowsMsg) {
			w.WriteHeader(http.StatusBadRequest)
			// ToJSON(&GenericError{Error: ErrUserNotFound}, w)
			ToJSON(&GenericResponse{Status: false, Message: ErrUserNotFound}, w)
		} else {
			w.WriteHeader(http.StatusInternalServerError)
			// ToJSON(&GenericError{Error: err.Error()}, w)
			ToJSON(&GenericResponse{Status: false, Message: "Unable to retrieve user from database. Please try again later"}, w)
		}
		return
	}

	if valid := ah.authService.Authenticate(&reqUser, user); !valid {
		ah.logger.Debug("Authetication of user failed")
		w.WriteHeader(http.StatusBadRequest)
		// ToJSON(&GenericError{Error: "incorrect password"}, w)
		ToJSON(&GenericResponse{Status: false, Message: "Incorrect password"}, w)
		return
	}

	accessToken, err := ah.authService.GenerateAccessToken(user)
	if err != nil {
		ah.logger.Error("unable to generate access token", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		// ToJSON(&GenericError{Error: err.Error()}, w)
		ToJSON(&GenericResponse{Status: false, Message: "Unable to login the user. Please try again later"}, w)
		return
	}
	refreshToken, err := ah.authService.GenerateRefreshToken(user)
	if err != nil {
		ah.logger.Error("unable to generate refresh token", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		// ToJSON(&GenericError{Error: err.Error()}, w)
		ToJSON(&GenericResponse{Status: false, Message: "Unable to login the user. Please try again later"}, w)
		return
	}

	ah.logger.Debug("successfully generated token ", "accesstoken: ", accessToken, " refreshtoken: ", refreshToken)
	w.WriteHeader(http.StatusOK)
	// ToJSON(&AuthResponse{AccessToken: accessToken, RefreshToken: refreshToken, Username: user.Username}, w)
	ToJSON(&GenericResponse{
		Status:  true,
		Message: "Successfully logged in",
		Data:    &AuthResponse{AccessToken: accessToken, RefreshToken: refreshToken, Username: user.Username},
	}, w)
}
