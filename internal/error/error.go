package error

import (
	"fmt"
)

// Throwed when certain labels cannot be found on a resource.
type LabelNotFoundError struct {
	Err               error
	LabelKey          string
	ResourceKind      string
	ResourceName      string
	ResourceNamespace string
}

func (r *LabelNotFoundError) Error() string {
	return fmt.Sprintf("label with key %v is not found in resource %v %v in the namespace %v", r.LabelKey, r.ResourceKind, r.ResourceName, r.ResourceNamespace)
}

type NodeNotFoundError struct {
	Err               error
	ResourceKind      string
	ResourceName      string
	ResourceNamespace string
}

func (r *NodeNotFoundError) Error() string {
	return fmt.Sprintf("no available node for resource %v %v in namespace %v", r.ResourceKind, r.ResourceName, r.ResourceNamespace)
}

type MultipleNodeFoundError struct {
	Err               error
	ResourceKind      string
	ResourceName      string
	ResourceNamespace string
}

func (r *MultipleNodeFoundError) Error() string {
	return fmt.Sprintf("multiple nodes are found for resource %v %v in namespace %v", r.ResourceKind, r.ResourceName, r.ResourceNamespace)
}
