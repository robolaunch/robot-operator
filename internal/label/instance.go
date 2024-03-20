package label

import metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"

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
