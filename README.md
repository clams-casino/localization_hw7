# Building

Build it on the duckiebot. Assuming in the root directory of the repo
```
dts devel build -f -H <duckiebot name>.local
```

# Running fused localization
Use the following command to run fused localization on the duckiebot
```
dts devel run -H <duckiebot name>.local
```
The default launcher is setup to start the `fused_localization_node`, `encoder_localization_node`, and `at_localization_node`.

# Running encoder localization
```
dts devel run -H <duckiebot name>.local --launcher encoder
```
This will only start the `encoder_localization_node`.

# Running april tag localization
```
dts devel run -H <duckiebot name>.local --launcher at
```
This will only start the `at_localization_node`.
