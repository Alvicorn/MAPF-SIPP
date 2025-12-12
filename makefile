.PHONY: test_all unit_test test_static_experiments_standard test_static_experiments_disjoint


STATIC_TEST_PATH = instances/tests

test_all:
	pytest -n 3 tests

unit_test:
	pytest --cov=. --cov-report term-missing tests/unit

# example usage of dynamic env_experiment (similar to the static env experiment)
run_dynamic_env_sample_experiment:
	python run_dynamic_experiments.py \
		--map instances/maps/custom_instances/simple-8-8.txt \
		--instance instances/dynamic_instances/custom_instances/simple-8-8-1.toml

test_static_experiments_standard:
	python run_static_experiments.py \
		--instance "$(STATIC_TEST_PATH)/test_*" \
		--batch \
		--test $(STATIC_TEST_PATH)/min-sum-of-cost.csv

test_static_experiments_disjoint:
	python run_static_experiments.py \
		--instance "$(STATIC_TEST_PATH)/test_*" \
		--batch \
		--disjoint \
		--test $(STATIC_TEST_PATH)/min-sum-of-cost.csv
