package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

type Timezone struct {
	Continent string
	City      string
}

func GetTimezone(obj metav1.Object) *Timezone {

	timezone := &Timezone{}
	labels := obj.GetLabels()

	if continent, ok := labels[internal.TZ_CONTINENT_LABEL_KEY]; ok {
		timezone.Continent = continent
	}

	if city, ok := labels[internal.TZ_CITY_LABEL_KEY]; ok {
		timezone.City = city
	}

	return timezone
}
