package label

import (
	"github.com/robolaunch/robot-operator/internal"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

type Tenancy struct {
	Organization     string
	Team             string
	Region           string
	BufferInstance   string
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

	if bufferInstance, ok := labels[internal.BUFFER_INSTANCE_LABEL_KEY]; ok {
		tenancy.BufferInstance = bufferInstance
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

	if bufferInstance, ok := labels[internal.BUFFER_INSTANCE_LABEL_KEY]; ok {
		tenancyMap[internal.BUFFER_INSTANCE_LABEL_KEY] = bufferInstance
	}

	if cloudInstance, ok := labels[internal.CLOUD_INSTANCE_LABEL_KEY]; ok {
		tenancyMap[internal.CLOUD_INSTANCE_LABEL_KEY] = cloudInstance
	}

	if physicalInstance, ok := labels[internal.PHYSICAL_INSTANCE_LABEL_KEY]; ok {
		tenancyMap[internal.PHYSICAL_INSTANCE_LABEL_KEY] = physicalInstance
	}

	return tenancyMap
}

func GetTenancyMapFromTenancy(tenancy Tenancy) map[string]string {
	tenancyMap := make(map[string]string)

	if tenancy.Organization != "" {
		tenancyMap[internal.ORGANIZATION_LABEL_KEY] = tenancy.Organization
	}

	if tenancy.Team != "" {
		tenancyMap[internal.TEAM_LABEL_KEY] = tenancy.Team
	}

	if tenancy.Region != "" {
		tenancyMap[internal.REGION_LABEL_KEY] = tenancy.Region
	}

	if tenancy.BufferInstance != "" {
		tenancyMap[internal.BUFFER_INSTANCE_LABEL_KEY] = tenancy.BufferInstance
	}

	if tenancy.CloudInstance != "" {
		tenancyMap[internal.CLOUD_INSTANCE_LABEL_KEY] = tenancy.CloudInstance
	}

	if tenancy.PhysicalInstance != "" {
		tenancyMap[internal.PHYSICAL_INSTANCE_LABEL_KEY] = tenancy.PhysicalInstance
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

func GetClusterName(obj metav1.Object) string {
	tenancy := GetTenancy(obj)
	if tenancy.PhysicalInstance == "" {
		return tenancy.CloudInstance
	}
	return tenancy.PhysicalInstance
}

func GetBufferInstanceName(obj metav1.Object) string {
	tenancy := GetTenancy(obj)
	return tenancy.BufferInstance
}
