package auth

import (
	"meteo-daemon/services/data"
	"net/http"
)

// RefreshToken handles refresh token request
func (ah *AuthHandler) RefreshToken(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	user := r.Context().Value(UserKey{}).(data.User)
	accessToken, err := ah.authService.GenerateAccessToken(&user)
	if err != nil {
		ah.logger.Error("unable to generate access token", "error", err)
		w.WriteHeader(http.StatusInternalServerError)
		// data.ToJSON(&GenericError{Error: err.Error()}, w)
		ToJSON(&GenericResponse{Status: false, Message: "Unable to generate access token.Please try again later"}, w)
		return
	}

	w.WriteHeader(http.StatusOK)
	// data.ToJSON(&TokenResponse{AccessToken: accessToken}, w)
	ToJSON(&GenericResponse{
		Status:  true,
		Message: "Successfully generated new access token",
		Data:    &TokenResponse{AccessToken: accessToken},
	}, w)
}

// Greet request greet request
func (ah *AuthHandler) Greet(w http.ResponseWriter, r *http.Request) {

	w.Header().Set("Content-Type", "application/json")

	userID := r.Context().Value(UserIDKey{}).(string)
	w.WriteHeader(http.StatusOK)
	// w.Write([]byte("hello, " + userID))
	ToJSON(&GenericResponse{
		Status:  true,
		Message: "hello," + userID,
	}, w)
}
