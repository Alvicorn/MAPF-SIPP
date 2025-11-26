.PHONY: test_all unit_test test_static_experiments_standard test_static_experiments_disjoint


STATIC_TEST_PATH = instances/tests

test_all:
	pytest -n 3 tests

unit_test:
	pytest --cov=. --cov-report term-missing tests/unit

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
