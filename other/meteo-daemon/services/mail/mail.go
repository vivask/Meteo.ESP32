package mail

import (
	"fmt"
	"meteo-daemon/domain"
	"meteo-daemon/utils"
	"os"

	"github.com/sendgrid/sendgrid-go"
	"github.com/sendgrid/sendgrid-go/helpers/mail"
	"github.com/sirupsen/logrus"
)

// MailService represents the interface for our mail service.
type MailService interface {
	CreateMail(mailReq *Mail) []byte
	SendMail(mailReq *Mail) error
	NewMail(from string, to []string, subject string, mailType MailType, data *MailData) *Mail
}

type MailType int

// List of Mail Types we are going to send.
const (
	MailConfirmation MailType = iota + 1
	PassReset
)

// MailData represents the data to be sent to the template of the mail.
type MailData struct {
	Username string
	Code     string
}

// Mail represents a email request
type Mail struct {
	from    string
	to      []string
	subject string
	body    string
	mtype   MailType
	data    *MailData
}

type Params struct {
	Hasp   domain.StartStopInterface
	Logger *logrus.Entry
}

// SGMailService is the sendgrid implementation of our MailService.
type SGMailService struct {
	hasp    domain.StartStopInterface
	logger  *logrus.Entry
	logfile *os.File
	config  *Config
}

// NewSGMailService returns a new instance of SGMailService
func New(cnf *Config, p *Params) *SGMailService {
	return &SGMailService{
		hasp:    p.Hasp,
		logger:  p.Logger,
		logfile: nil,
		config:  cnf,
	}
}

// CreateMail takes in a mail request and constructs a sendgrid mail type.
func (ms *SGMailService) CreateMail(mailReq *Mail) []byte {

	m := mail.NewV3Mail()

	from := mail.NewEmail("bookite", mailReq.from)
	m.SetFrom(from)

	if mailReq.mtype == MailConfirmation {
		m.SetTemplateID(ms.config.MailVerifTemplateID)
	} else if mailReq.mtype == PassReset {
		m.SetTemplateID(ms.config.PassResetTemplateID)
	}

	p := mail.NewPersonalization()

	tos := make([]*mail.Email, 0)
	for _, to := range mailReq.to {
		tos = append(tos, mail.NewEmail("user", to))
	}

	p.AddTos(tos...)

	p.SetDynamicTemplateData("Username", mailReq.data.Username)
	p.SetDynamicTemplateData("Code", mailReq.data.Code)

	m.AddPersonalizations(p)
	return mail.GetRequestBody(m)
}

// SendMail creates a sendgrid mail from the given mail request and sends it.
func (ms *SGMailService) SendMail(mailReq *Mail) error {

	endpoint := "/3.0/lists/" + ms.config.MailChipListId + "/members/"
	request := sendgrid.GetRequest(ms.config.MailChimpApiKey, endpoint, "https://us14.api.mailchimp.com")
	request.Method = "POST"
	var Body = ms.CreateMail(mailReq)
	request.Body = Body
	response, err := sendgrid.API(request)
	if err != nil {
		ms.logger.Error("unable to send mail", "error", err)
		return err
	}
	ms.logger.Info("mail sent successfully", "sent status code", response.StatusCode)
	return nil
}

// NewMail returns a new mail request.
func (ms *SGMailService) NewMail(from string, to []string, subject string, mailType MailType, data *MailData) *Mail {
	return &Mail{
		from:    from,
		to:      to,
		subject: subject,
		mtype:   mailType,
		data:    data,
	}
}

func (SGMailService) StartNum() int {
	return 3
}

func (ms *SGMailService) Enabled() bool {
	return ms.config.Active
}
func (ms *SGMailService) IsRun() bool {
	return ms.hasp.IsRun()
}

func (ms *SGMailService) Start() error {
	if !ms.hasp.Start() {
		return fmt.Errorf("%s:failed to start", ms.config.Title)
	}

	ms.logger.Debugf("%s: success started", ms.config.Title)

	return nil
}

func (ms *SGMailService) Stop() error {
	if !ms.hasp.Stop() {
		return fmt.Errorf("%s:failed to stop", ms.config.Title)
	}

	ms.logger.Debugf("%s: success stoped", ms.config.Title)

	if ms.logfile != nil {
		ms.logfile.Close()
	}

	return nil
}

func (ms *SGMailService) InitLog(logDir string, file *os.File) error {
	f, err := utils.InitLogrus(logDir, ms.config.LogFile, ms.config.LogLevel, file, ms.logger)
	if err != nil {
		return err
	}
	ms.logfile = f
	return nil
}

type Config struct {
	Title                   string `toml:"Title"`
	Active                  bool   `toml:"Active"`
	MailChimpApiKey         string `toml:"MailChimpApiKey"`
	MailChipListId          string `toml:"MailChipListId"`
	MailVerifCodeExpiration int    `toml:"MailVerifCodeExpiration"` // in hours
	PassResetCodeExpiration int    `toml:"PassResetCodeExpiration"` // in minutes
	MailVerifTemplateID     string `toml:"MailVerifTemplateID"`
	PassResetTemplateID     string `toml:"PassResetTemplateID"`
	LogFile                 string `toml:"LogFile"`
	LogLevel                string `toml:"LogLevel"`
}
