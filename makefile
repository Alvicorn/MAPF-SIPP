.PHONY: test_static_experiments_standard test_static_experiments_disjoint


STATIC_TEST_PATH = instances/static/tests


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
