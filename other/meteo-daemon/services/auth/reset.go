package auth

import (
	"context"
	"meteo-daemon/services/data"
	"meteo-daemon/utils"
	"net/http"
)

// UpdateUsername handles username update request
func (ah *AuthHandler) UpdateUsername(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	user := &data.User{}
	err := FromJSON(user, r.Body)
	if err != nil {
		ah.logger.Error("unable to decode user json", "error", err.Error())
		w.WriteHeader(http.StatusBadRequest)
		// data.ToJSON(&GenericError{Error: err.Error()}, w)
		ToJSON(&GenericResponse{Status: false, Message: err.Error()}, w)
		return
	}

	user.ID = r.Context().Value(UserIDKey{}).(string)
	ah.logger.Debug("udpating username for user : ", user)

	err = ah.repo.UpdateUsername(context.Background(), user)
	if err != nil {
		ah.logger.Error("unable to update username", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		// data.ToJSON(&GenericError{Error: err.Error()}, w)
		ToJSON(&GenericResponse{Status: false, Message: "Unable to update username. Please try again later"}, w)
		return
	}

	w.WriteHeader(http.StatusOK)
	// data.ToJSON(&UsernameUpdate{Username: user.Username}, w)
	ToJSON(&GenericResponse{
		Status:  true,
		Message: "Successfully updated username",
		Data:    &UsernameUpdate{Username: user.Username},
	}, w)
}

// PasswordReset handles the password reset request
func (ah *AuthHandler) ResetPassword(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	passResetReq := &PasswordResetReq{}
	err := FromJSON(passResetReq, r.Body)
	if err != nil {
		ah.logger.Error("unable to decode password reset request json", "error", err.Error())
		w.WriteHeader(http.StatusBadRequest)
		ToJSON(&GenericResponse{Status: false, Message: err.Error()}, w)
		return
	}

	userID := r.Context().Value(UserIDKey{}).(string)
	_, err = ah.repo.GetUserByID(context.Background(), userID)
	if err != nil {
		ah.logger.Error("unable to retrieve the user from db", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		ToJSON(&GenericResponse{Status: false, Message: "Unable to reset password. Please try again later"}, w)
		return
	}

	if passResetReq.Password != passResetReq.PasswordRe {
		ah.logger.Error("password and password re-enter did not match")
		w.WriteHeader(http.StatusNotAcceptable)
		ToJSON(&GenericResponse{Status: false, Message: "Password and re-entered Password are not same"}, w)
		return
	}

	hashedPass, err := ah.hashPassword(passResetReq.Password)
	if err != nil {
		w.WriteHeader(http.StatusInternalServerError)
		ToJSON(&GenericResponse{Status: false, Message: UserCreationFailed}, w)
		return
	}

	tokenHash := utils.GenerateRandomString(15)
	err = ah.repo.UpdatePassword(context.Background(), userID, hashedPass, tokenHash)
	if err != nil {
		ah.logger.Error("unable to update user password in db", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		ToJSON(&GenericResponse{Status: false, Message: "Password and re-entered Password are not same"}, w)
		return
	}

	w.WriteHeader(http.StatusOK)
	ToJSON(&GenericResponse{Status: false, Message: "Password Reset Successfully"}, w)
}
