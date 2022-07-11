package proxy

import (
	"context"
	"fmt"
	"io"
	"io/ioutil"
	"meteo-daemon/services/data"
	"net/http"
	"os"
	"regexp"
	"strings"
	s "strings"

	"github.com/sirupsen/logrus"
)

const blockListFileName = "blocklist.txt"

type BlackList struct {
	logger *logrus.Entry
	data   map[string]struct{}
}

func NewBlackList(lg *logrus.Entry) *BlackList {

	return &BlackList{
		logger: lg,
		data:   make(map[string]struct{}),
	}
}

func Split(r rune) bool {
	return r == ' ' || r == '\t'
}

func (b *BlackList) Count() int {
	return len(b.data)
}

func (b *BlackList) Add(host string) bool {

	if _, exist := b.data[host]; !exist {
		b.data[host] = struct{}{}
		return true
	}
	return false
}

func extractName(line string) (name string, good bool) {

	if len(line) == 0 || line[0:1] == "#" || line[0:1] == "!" || badHost(line) {
		return "", false
	}

	if len(line) > 2 && line[0:2] == "||" {
		name = line[2 : len(line)-1]
		if !s.HasSuffix(name, ".") {
			name += "."
		}

		return name, true
	}

	if len(line) > 10 && (line[0:9] == "127.0.0.1" || line[0:7] == "0.0.0.0") {
		split := s.FieldsFunc(line, Split)
		name = s.Trim(split[1], " ")
		if len(name) == 0 {
			return "", false
		}

		if !s.HasSuffix(name, ".") {
			name += "."
		}

		return name, true
	} else {
		return "", false
	}
}

func badHost(host string) bool {

	badhosts := []string{
		"localhost",
		"localhost.localdomain",
		"broadcasthost",
		"local",
		"ip6-localhost",
		"ip6-loopback",
		"ip6-localnet",
		"ip6-mcastprefix",
		"ip6-allnodes",
		"ip6-allrouters",
		"ip6-allhosts",
	}

	for _, bad := range badhosts {
		matched, _ := regexp.MatchString(bad, host)
		if matched {
			return true
		}
	}
	return false
}

func (b *BlackList) AddList(lines []string, idx int) (count int) {

	for _, line := range lines {
		name, ok := extractName(line)
		if ok && b.Add(name) {
			count++
		}
	}

	return
}

func (b *BlackList) loadFromDb(repo data.Repository) (count int) {

	hosts, err := repo.GetAllBlockHosts(context.Background())
	if err != nil {
		b.logger.Error(err)
	}
	for _, host := range hosts {
		b.data[host.ID] = struct{}{}
		count++
	}
	return
}

func (b *BlackList) saveToDb(repo data.Repository) error {
	ctx := context.Background()
	err := repo.ClearBlocklist(ctx)
	if err != nil {
		return fmt.Errorf("clear blocklist error: %w", err)
	}

	for host := range b.data {
		err := repo.AddBlockHost(ctx, data.Blocklist{ID: host})
		if err != nil {
			return fmt.Errorf("add blocklist error: %w", err)
		}
	}
	return nil
}

func (b *BlackList) loadFromFile(dir string) (count int, err error) {
	fName := fmt.Sprintf("%s/%s", dir, blockListFileName)
	f, err := os.OpenFile(fName, os.O_RDONLY|os.O_CREATE, 0644)
	if err != nil {
		return 0, fmt.Errorf("open file error: %w", err)
	}
	defer f.Close()
	buf := new(strings.Builder)
	io.Copy(buf, f)
	hosts := strings.Split(buf.String(), "\n")
	for _, host := range hosts {
		if len(strings.Trim(host, " ")) != 0 {
			b.data[host] = struct{}{}
			count++
		}
	}
	return
}

func (b *BlackList) SaveToFile(dir string) error {
	fName := fmt.Sprintf("%s/%s", dir, blockListFileName)
	f, err := os.OpenFile(fName, os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		return fmt.Errorf("open file error: %w", err)
	}
	defer f.Close()
	for k := range b.data {
		_, err = io.WriteString(f, fmt.Sprintf("%s\n", k))
		if err != nil {
			return fmt.Errorf("file write error: %w", err)
		}
	}
	return nil
}

func (b *BlackList) Contains(server string) bool {
	_, ok := b.data[server]
	return ok
}

func updateList(lg *logrus.Entry, lists []string) (list *BlackList, loaded []bool) {

	list = NewBlackList(lg)
	loaded = make([]bool, len(lists))

	for idx, v := range lists {
		resp, err := http.Get(v)
		if err != nil {
			lg.Println("[black] Can't load", v)
			continue
		}

		if resp.StatusCode != 200 {
			lg.Println("[black] Status code of", v, "!= 200")
			continue
		}

		data, err := ioutil.ReadAll(resp.Body)
		if err != nil {
			lg.Println("[black] Can't read body of", v)
			continue
		}

		data2 := s.Split(string(data), "\n")
		cnt := list.AddList(data2, idx)

		if cnt == 0 {
			loaded[idx] = false
			lg.Errorf("No access to block list: %s", v)
		} else {
			loaded[idx] = true
			lg.Infof("Uploaded %d blocked hosts from: %s", cnt, v)
		}
	}
	return
}

func UpdateDb(lg *logrus.Entry, lists []string, repo data.Repository) (list *BlackList, loaded []bool) {

	list, loaded = updateList(lg, lists)

	err := list.saveToDb(repo)
	if err != nil {
		lg.Error(err)
	}

	lg.Infof("Loaded %d block hosts from database", list.Count())

	return
}

func LoadFromDb(lg *logrus.Entry, repo data.Repository) *BlackList {

	list := NewBlackList(lg)
	cnt := list.loadFromDb(repo)
	lg.Infof("Loaded %d block hosts from database", cnt)

	return list
}

func UpdateFile(lg *logrus.Entry, lists []string, dir string) (list *BlackList, loaded []bool) {

	list, loaded = updateList(lg, lists)

	err := list.SaveToFile(dir)
	if err != nil {
		lg.Error(err)
	}

	lg.Infof("Loaded %d block hosts from file", list.Count())

	return
}

func LoadFromFile(lg *logrus.Entry, dir string) *BlackList {
	list := NewBlackList(lg)
	cnt, err := list.loadFromFile(dir)
	if err != nil {
		lg.Errorf("load block list from file error: %w", err)
		return list
	}
	lg.Infof("Loaded %d block hosts from file", cnt)

	return list
}
