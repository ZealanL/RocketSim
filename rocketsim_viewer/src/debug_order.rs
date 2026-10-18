#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub(crate) enum DrawLayer {
    Transparent,
    XrayHidden,
    XrayVisible,
    Overlay,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct OrderedInstance {
    pub material: usize,
    pub layer: DrawLayer,
    pub view_depth: f32,
    pub source_order: (usize, usize),
    pub transform: glam::Mat4,
    pub tint: [f32; 4],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct InstanceRun {
    pub material: usize,
    pub layer: DrawLayer,
    pub first: usize,
    pub count: usize,
}

pub(crate) fn sort_back_to_front(instances: &mut [OrderedInstance]) {
    instances.sort_by(|a, b| {
        a.layer
            .cmp(&b.layer)
            .then_with(|| b.view_depth.total_cmp(&a.view_depth))
            .then_with(|| a.source_order.cmp(&b.source_order))
            .then_with(|| a.material.cmp(&b.material))
    });
}

pub(crate) fn contiguous_runs(instances: &[OrderedInstance]) -> Vec<InstanceRun> {
    let mut runs: Vec<InstanceRun> = Vec::new();

    for (index, instance) in instances.iter().enumerate() {
        if let Some(run) = runs.last_mut()
            && run.material == instance.material
            && run.layer == instance.layer
        {
            run.count += 1;
            continue;
        }

        runs.push(InstanceRun {
            material: instance.material,
            layer: instance.layer,
            first: index,
            count: 1,
        });
    }

    runs
}

#[cfg(test)]
mod tests {
    use super::{DrawLayer, InstanceRun, OrderedInstance, contiguous_runs, sort_back_to_front};
    use glam::Mat4;

    fn instance(
        material: usize,
        layer: DrawLayer,
        depth: f32,
        order: (usize, usize),
    ) -> OrderedInstance {
        OrderedInstance {
            material,
            layer,
            view_depth: depth,
            source_order: order,
            transform: Mat4::IDENTITY,
            tint: [1.0; 4],
        }
    }

    fn identity(instance: &OrderedInstance) -> (DrawLayer, u32, (usize, usize), usize) {
        (
            instance.layer,
            instance.view_depth.to_bits(),
            instance.source_order,
            instance.material,
        )
    }

    #[test]
    fn sorts_all_input_permutations_deterministically() {
        let items = [
            instance(2, DrawLayer::Transparent, 3.0, (1, 0)),
            instance(1, DrawLayer::Transparent, 8.0, (0, 2)),
            instance(0, DrawLayer::XrayHidden, 100.0, (2, 0)),
            instance(3, DrawLayer::Overlay, 1.0, (0, 0)),
        ];
        let mut expected = None;

        for a in 0..items.len() {
            for b in 0..items.len() {
                for c in 0..items.len() {
                    for d in 0..items.len() {
                        if [a, b, c, d]
                            .iter()
                            .copied()
                            .collect::<std::collections::HashSet<_>>()
                            .len()
                            != 4
                        {
                            continue;
                        }
                        let mut permutation = [items[a], items[b], items[c], items[d]];
                        sort_back_to_front(&mut permutation);
                        let result: Vec<_> = permutation.iter().map(identity).collect();
                        if let Some(expected) = &expected {
                            assert_eq!(&result, expected);
                        } else {
                            expected = Some(result);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn sorts_layers_then_depth_back_to_front() {
        let mut items = [
            instance(0, DrawLayer::XrayVisible, 10.0, (0, 0)),
            instance(0, DrawLayer::Transparent, 2.0, (0, 1)),
            instance(0, DrawLayer::Transparent, 9.0, (0, 0)),
            instance(0, DrawLayer::XrayHidden, 50.0, (0, 2)),
            instance(0, DrawLayer::Overlay, 100.0, (0, 3)),
        ];

        sort_back_to_front(&mut items);

        assert_eq!(
            items.iter().map(|item| item.layer).collect::<Vec<_>>(),
            [
                DrawLayer::Transparent,
                DrawLayer::Transparent,
                DrawLayer::XrayHidden,
                DrawLayer::XrayVisible,
                DrawLayer::Overlay,
            ]
        );
        assert_eq!(items[0].view_depth, 9.0);
        assert_eq!(items[1].view_depth, 2.0);
    }

    #[test]
    fn equal_depth_uses_source_order_then_material() {
        let mut items = [
            instance(2, DrawLayer::Transparent, 4.0, (1, 0)),
            instance(1, DrawLayer::Transparent, 4.0, (0, 1)),
            instance(3, DrawLayer::Transparent, 4.0, (0, 0)),
            instance(0, DrawLayer::Transparent, 4.0, (0, 1)),
        ];

        sort_back_to_front(&mut items);

        assert_eq!(items.map(|item| item.material), [3, 0, 1, 2]);
    }

    #[test]
    fn runs_merge_only_adjacent_matching_material_and_layer() {
        let items = [
            instance(1, DrawLayer::Transparent, 3.0, (0, 0)),
            instance(2, DrawLayer::Transparent, 2.0, (0, 1)),
            instance(1, DrawLayer::Transparent, 1.0, (0, 2)),
            instance(1, DrawLayer::Transparent, 0.0, (0, 3)),
            instance(1, DrawLayer::XrayHidden, 0.0, (0, 4)),
            instance(1, DrawLayer::XrayHidden, 0.0, (0, 5)),
        ];

        assert_eq!(
            contiguous_runs(&items),
            [
                InstanceRun {
                    material: 1,
                    layer: DrawLayer::Transparent,
                    first: 0,
                    count: 1
                },
                InstanceRun {
                    material: 2,
                    layer: DrawLayer::Transparent,
                    first: 1,
                    count: 1
                },
                InstanceRun {
                    material: 1,
                    layer: DrawLayer::Transparent,
                    first: 2,
                    count: 2
                },
                InstanceRun {
                    material: 1,
                    layer: DrawLayer::XrayHidden,
                    first: 4,
                    count: 2
                },
            ]
        );
    }

    #[test]
    fn empty_input_has_no_runs() {
        assert!(contiguous_runs(&[]).is_empty());
    }
}
