package data

import (
	"github.com/jinzhu/gorm"
)

func (j *Jobs) BeforeDelete(tx *gorm.DB) (err error) {
	return tx.Delete(&JobParams{}, "job_id = ?", j.ID).Error
}

func (t *Tasks) BeforeDelete(tx *gorm.DB) (err error) {
	return tx.Delete(&TaskParams{}, "task_id = ?", t.ID).Error
}

func (t *SyncTables) BeforeDelete(tx *gorm.DB) (err error) {
	return tx.Delete(&SyncParams{}, "table_id = ?", t.ID).Error
}
