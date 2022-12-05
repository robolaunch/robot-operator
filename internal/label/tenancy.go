package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

type Tenancy struct {
	Organization     string
	Team             string
	Region           string
	CloudInstance    string
	PhysicalInstance string
}

func GetTenancy(obj metav1.Object) *Tenancy {
	tenancy := &Tenancy{}
	labels := obj.GetLabels()

	if organization, ok := labels[internal.ORGANIZATION_LABEL_KEY]; ok {
		tenancy.Organization = organization
	}

	if team, ok := labels[internal.TEAM_LABEL_KEY]; ok {
		tenancy.Team = team
	}

	if region, ok := labels[internal.REGION_LABEL_KEY]; ok {
		tenancy.Region = region
	}

	if cloudInstance, ok := labels[internal.CLOUD_INSTANCE_LABEL_KEY]; ok {
		tenancy.CloudInstance = cloudInstance
	}

	if physicalInstance, ok := labels[internal.PHYSICAL_INSTANCE_LABEL_KEY]; ok {
		tenancy.PhysicalInstance = physicalInstance
	}

	return tenancy
}

func GetTenancyMap(obj metav1.Object) map[string]string {
	labels := obj.GetLabels()
	tenancyMap := make(map[string]string)

	if organization, ok := labels[internal.ORGANIZATION_LABEL_KEY]; ok {
		tenancyMap[internal.ORGANIZATION_LABEL_KEY] = organization
	}

	if team, ok := labels[internal.TEAM_LABEL_KEY]; ok {
		tenancyMap[internal.TEAM_LABEL_KEY] = team
	}

	if region, ok := labels[internal.REGION_LABEL_KEY]; ok {
		tenancyMap[internal.REGION_LABEL_KEY] = region
	}

	if cloudInstance, ok := labels[internal.CLOUD_INSTANCE_LABEL_KEY]; ok {
		tenancyMap[internal.CLOUD_INSTANCE_LABEL_KEY] = cloudInstance
	}

	if physicalInstance, ok := labels[internal.PHYSICAL_INSTANCE_LABEL_KEY]; ok {
		tenancyMap[internal.PHYSICAL_INSTANCE_LABEL_KEY] = physicalInstance
	}

	return tenancyMap
}

type InstanceType string

const (
	InstanceTypeCloudInstance    InstanceType = "CloudInstance"
	InstanceTypePhysicalInstance InstanceType = "PhysicalInstance"
)

func GetInstanceType(obj metav1.Object) InstanceType {
	tenancy := GetTenancy(obj)
	if tenancy.PhysicalInstance == "" {
		return InstanceTypeCloudInstance
	}
	return InstanceTypePhysicalInstance
}
