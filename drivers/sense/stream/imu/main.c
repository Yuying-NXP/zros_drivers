/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <zros/perf_duration.h>

#include <synapse_topic_list.h>

#include <zros/private/zros_node_struct.h>
#include <zros/private/zros_pub_struct.h>
#include <zros/private/zros_sub_struct.h>
#include <zros/zros_node.h>
#include <zros/zros_pub.h>
#include <zros/zros_sub.h>

LOG_MODULE_REGISTER(sense_imu_stream, CONFIG_ZROS_SENSE_STREAM_IMU_LOG_LEVEL);

/** Borrowed from zephyr/dsp/util.h as it was erroring out due to a missing
 * zdsp backend. Should sort that out.
 */
#define Z_SHIFT_Q31_TO_F32(src, m) ((float)(((int64_t)src) << m) / (float)(1U << 31))

#define ACCEL_G ((float)SENSOR_G / 1000000.0f)

#define IMU_STREAM_CALIBRATION_COUNT 1000
#define IMU_ALIAS(i) DT_ALIAS(_CONCAT(imu_stream_,i))

/*
 * 2nd-order Butterworth LPF coefficients, Direct Form I
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */
struct iir2_coeffs {
	float b0, b1, b2, a1, a2;
};

/* Accel LPF: fc=30 Hz, fs=800 Hz — filters vibration for the attitude estimator */
static const struct iir2_coeffs accel_lpf = {
	.b0 =  0.011858f, .b1 =  0.023715f, .b2 =  0.011858f,
	.a1 = -1.669203f, .a2 =  0.716634f,
};

/* Gyro LPF: fc=120 Hz, fs=800 Hz — minimal phase lag for the rate controller */
// static const struct iir2_coeffs gyro_lpf = {
// 	.b0 =  0.131106f, .b1 =  0.262213f, .b2 =  0.131106f,
// 	.a1 = -0.747789f, .a2 =  0.272215f,
// };


/* Gyro LPF: fc=15 Hz, 2nd order for buggy */
static const struct iir2_coeffs gyro_lpf = {
	.b0 =  0.00320077, .b1 =  0.00640154, .b2 =  0.00320077,
	.a1 = -1.83370725  , .a2 =  0.84651034,
};

struct iir2_state {
	float x1, x2;  /* previous inputs */
	float y1, y2;  /* previous outputs */
};

static inline float iir2_update(struct iir2_state *s, const struct iir2_coeffs *c, float x)
{
	float y = c->b0 * x + c->b1 * s->x1 + c->b2 * s->x2
		  - c->a1 * s->y1 - c->a2 * s->y2;
	s->x2 = s->x1;
	s->x1 = x;
	s->y2 = s->y1;
	s->y1 = y;
	return y;
}

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev);

enum sense_imu_stream_calibration_st {
	SENSE_IMU_STREAM_UNCALIBRATED,
	SENSE_IMU_STREAM_CALIBRATING,
	SENSE_IMU_STREAM_CALIBRATED,
};

struct context {
	const char *name;
	struct zros_node node;
	struct zros_pub pub_imu;
	struct zros_topic *pub_topic;
	synapse_pb_Imu imu;
	struct zros_sub sub_status;
	synapse_pb_Status status;
	synapse_pb_Status_Mode last_mode;
	bool running;
	struct {
		enum sense_imu_stream_calibration_st state;
		float accel_scale;
		struct {
			float accel[IMU_STREAM_CALIBRATION_COUNT][3];
			float gyro[IMU_STREAM_CALIBRATION_COUNT][3];
		} samples;
		struct {
			float accel[3];
			float gyro[3];
		} bias;
		size_t count;
	} calibration;
	struct {
		float accel[3];
		float gyro[3];
	} filtered;
	struct {
		struct iir2_state accel[3];
		struct iir2_state gyro[3];
	} filter;
	struct {
		struct rtio *ctx;
		struct rtio_iodev *iodev;
	} stream;
};

static void filter_init(struct context *ctx)
{
	memset(&ctx->filter, 0, sizeof(ctx->filter));
	memset(&ctx->filtered, 0, sizeof(ctx->filtered));
}

static void feed_calibration(struct context *ctx)
{
	if (ctx->calibration.state != SENSE_IMU_STREAM_CALIBRATING) {
		memset(&ctx->calibration.bias, 0, sizeof(ctx->calibration.bias));
		ctx->calibration.count = 0;
		ctx->calibration.state = SENSE_IMU_STREAM_CALIBRATING;
	}

	size_t idx = ctx->calibration.count;

	for (int i = 0; i < 3; i++) {
		ctx->calibration.samples.accel[idx][i] = ctx->filtered.accel[i];
		ctx->calibration.samples.gyro[idx][i] = ctx->filtered.gyro[i];
	}
	ctx->calibration.count++;

	if (ctx->calibration.count < IMU_STREAM_CALIBRATION_COUNT) {
		return;
	}

	float accel_mean[3] = {0};
	float gyro_mean[3] = {0};
	float accel_std[3] = {0};
	float gyro_std[3] = {0};
	bool calibration_ok = true;
	const float inv_count = 1.0f / IMU_STREAM_CALIBRATION_COUNT;

	for (size_t i = 0; i < IMU_STREAM_CALIBRATION_COUNT; i++) {
		for (int j = 0; j < 3; j++) {
			accel_mean[j] += ctx->calibration.samples.accel[i][j] * inv_count;
			gyro_mean[j] += ctx->calibration.samples.gyro[i][j] * inv_count;
		}
	}

	for (size_t i = 0; i < IMU_STREAM_CALIBRATION_COUNT; i++) {
		for (int j = 0; j < 3; j++) {
			float ea = ctx->calibration.samples.accel[i][j] - accel_mean[j];
			float eg = ctx->calibration.samples.gyro[i][j] - gyro_mean[j];

			accel_std[j] += ea * ea;
			gyro_std[j] += eg * eg;
		}
	}

	for (int i = 0; i < 3; i++) {
		accel_std[i] = sqrtf(accel_std[i] * inv_count);
		gyro_std[i] = sqrtf(gyro_std[i] * inv_count);
	}

	float accel_magnitude = sqrtf(accel_mean[0] * accel_mean[0] +
					  accel_mean[1] * accel_mean[1] +
					  accel_mean[2] * accel_mean[2]);

	if (accel_magnitude < 8.0f || accel_magnitude > 11.0f) {
		LOG_WRN_RATELIMIT_RATE(5000,
			"%s: accel magnitude out of range: %10.4f (expected ~9.8)",
			ctx->name, (double)accel_magnitude);
		calibration_ok = false;
	}

	for (int i = 0; i < 3; i++) {
		if (gyro_std[i] > 0.1f) {
			LOG_WRN_RATELIMIT_RATE(5000, "%s: gyro axis %d too noisy: std=%10.4f",
				ctx->name, i, (double)gyro_std[i]);
			calibration_ok = false;
		}
	}

	if (!calibration_ok) {
		LOG_WRN_RATELIMIT_RATE(5000, "%s: Calibration failed. Retrying...", ctx->name);
		ctx->calibration.state = SENSE_IMU_STREAM_UNCALIBRATED;
		return;
	}

	ctx->calibration.bias.accel[0] = accel_mean[0];
	ctx->calibration.bias.accel[1] = accel_mean[1];
	ctx->calibration.bias.accel[2] = 0;
	ctx->calibration.accel_scale = accel_magnitude / ACCEL_G;

	LOG_INF("%s: Calibration completed (scale=%.3f)", ctx->name,
		(double)ctx->calibration.accel_scale);
	LOG_DBG("%s: accel mean: %7.4f %7.4f %7.4f std: %7.4f %7.4f %7.4f", ctx->name,
		(double)accel_mean[0], (double)accel_mean[1], (double)accel_mean[2],
		(double)accel_std[0], (double)accel_std[1], (double)accel_std[2]);
	LOG_DBG("%s: gyro  mean: %7.4f %7.4f %7.4f std: %7.4f %7.4f %7.4f", ctx->name,
		(double)gyro_mean[0], (double)gyro_mean[1], (double)gyro_mean[2],
		(double)gyro_std[0], (double)gyro_std[1], (double)gyro_std[2]);

	for (int i = 0; i < 3; i++) {
		ctx->calibration.bias.gyro[i] = gyro_mean[i];
	}

	ctx->calibration.state = SENSE_IMU_STREAM_CALIBRATED;
}

static inline const struct sensor_decoder_api *get_imu_decoder(struct rtio_iodev *iodev)
{
	struct sensor_read_config *read_config = (struct sensor_read_config *)iodev->data;
	const struct device *sensor = read_config->sensor;
	const struct sensor_decoder_api *decoder;

	if (sensor_get_decoder(sensor, &decoder) < 0) {
		return NULL;
	}
	return decoder;
}

static inline const struct sensor_three_axis_ref *get_axis_ref(struct rtio_iodev *iodev)
{
	struct sensor_read_config *read_config = (struct sensor_read_config *)iodev->data;
	const struct device *sensor = read_config->sensor;
	const struct sensor_three_axis_ref *axis_ref;

	if (sensor_three_axis_ref_get(sensor, &axis_ref) < 0) {
		return NULL;
	}
	return axis_ref;
}

#define MAX_FIFO_FRAMES 16

static int decode_and_filter(struct context *ctx, uint8_t *buf)
{
	struct rtio_iodev *iodev = ctx->stream.iodev;
	const struct sensor_decoder_api *decoder = get_imu_decoder(iodev);
	const struct sensor_three_axis_ref *axis_ref = get_axis_ref(iodev);

	if (!decoder) {
		return -EIO;
	}

	/* Batch decode buffer — holds up to MAX_FIFO_FRAMES readings */
	uint8_t dec_buf[sizeof(struct sensor_three_axis_data) +
			(MAX_FIFO_FRAMES - 1) * sizeof(struct sensor_three_axis_sample_data)];
	struct sensor_three_axis_data *data = (struct sensor_three_axis_data *)dec_buf;
	uint32_t fit;
	int n;

	/* Decode all accel frames in one call */
	fit = 0;
	n = decoder->decode(buf, (struct sensor_chan_spec){SENSOR_CHAN_ACCEL_XYZ, 0},
			    &fit, MAX_FIFO_FRAMES, dec_buf);
	if (n > 0) {
		data->header.reading_count = n;
		if (axis_ref) {
			sensor_three_axis_ref_align(axis_ref, data);
		}
		for (int f = 0; f < n; f++) {
			for (int i = 0; i < 3; i++) {
				float sample = Z_SHIFT_Q31_TO_F32(
					data->readings[f].values[i], data->shift);
				ctx->filtered.accel[i] = iir2_update(
					&ctx->filter.accel[i], &accel_lpf, sample);
			}
		}
	}

	/* Decode all gyro frames in one call */
	fit = 0;
	n = decoder->decode(buf, (struct sensor_chan_spec){SENSOR_CHAN_GYRO_XYZ, 0},
			    &fit, MAX_FIFO_FRAMES, dec_buf);
	if (n > 0) {
		data->header.reading_count = n;
		if (axis_ref) {
			sensor_three_axis_ref_align(axis_ref, data);
		}
		for (int f = 0; f < n; f++) {
			for (int i = 0; i < 3; i++) {
				float sample = Z_SHIFT_Q31_TO_F32(
					data->readings[f].values[i], data->shift);
				ctx->filtered.gyro[i] = iir2_update(
					&ctx->filter.gyro[i], &gyro_lpf, sample);
			}
		}
	}

	return 0;
}

static void imu_publish(struct context *ctx)
{
	float inv_scale = 1.0f / ctx->calibration.accel_scale;

	stamp_msg(&ctx->imu.stamp, k_uptime_ticks());
	ctx->imu.linear_acceleration.x =
		(double)((ctx->filtered.accel[0] - ctx->calibration.bias.accel[0]) * inv_scale);
	ctx->imu.linear_acceleration.y =
		(double)((ctx->filtered.accel[1] - ctx->calibration.bias.accel[1]) * inv_scale);
	ctx->imu.linear_acceleration.z =
		(double)((ctx->filtered.accel[2] - ctx->calibration.bias.accel[2]) * inv_scale);
	ctx->imu.angular_velocity.x =
		(double)(ctx->filtered.gyro[0] - ctx->calibration.bias.gyro[0]);
	ctx->imu.angular_velocity.y =
		(double)(ctx->filtered.gyro[1] - ctx->calibration.bias.gyro[1]);
	ctx->imu.angular_velocity.z =
		(double)(ctx->filtered.gyro[2] - ctx->calibration.bias.gyro[2]);

	zros_pub_update(&ctx->pub_imu);
}

static inline bool calibration_requested(struct context *ctx)
{
	bool requested = ctx->status.mode == synapse_pb_Status_Mode_MODE_CALIBRATION &&
			 ctx->last_mode != synapse_pb_Status_Mode_MODE_CALIBRATION;

	ctx->last_mode = ctx->status.mode;
	return requested;
}

static void process_events(int result, uint8_t *buf, uint32_t len, void *userdata)
{
	struct rtio_iodev *iodev = (struct rtio_iodev *)userdata;
	struct context *ctx = get_context_from_iodev(iodev);

	if (result < 0) {
		LOG_WRN_RATELIMIT_RATE(5000, "%s: RTIO error: %d", ctx->name, result);
		return;
	}

	int ret = decode_and_filter(ctx, buf);

	if (ret < 0) {
		LOG_WRN_RATELIMIT_RATE(5000, "%s: decode error: %d", ctx->name, ret);
		return;
	}

	if (zros_sub_update_available(&ctx->sub_status)) {
		zros_sub_update(&ctx->sub_status);
	}

	if (calibration_requested(ctx) || ctx->calibration.state != SENSE_IMU_STREAM_CALIBRATED) {
		feed_calibration(ctx);
		return;
	}

	imu_publish(ctx);
}

static void drain_pending_cqes(struct context *ctx)
{
	struct rtio_cqe *cqe;
	uint8_t *buf = NULL;
	uint32_t buf_len = 0;

	while ((cqe = rtio_cqe_consume(ctx->stream.ctx)) != NULL) {
		rtio_cqe_get_mempool_buffer(ctx->stream.ctx, cqe, &buf, &buf_len);
		rtio_cqe_release(ctx->stream.ctx, cqe);
		rtio_release_buffer(ctx->stream.ctx, buf, buf_len);
	}
}

static int setup_stream(struct context *ctx)
{
	/* Configure IMUs to 800-Hz, if they haven't been configured in DTS */
	uint64_t period_ticks = (uint64_t)CONFIG_SYS_CLOCK_TICKS_PER_SEC * 1250 / 1000 / 1000;
	struct sensor_value ticks_per_event = {
		.val1 = period_ticks,
	};
	struct rtio *rtio_ctx = ctx->stream.ctx;
	struct rtio_iodev *iodev = ctx->stream.iodev;
	struct sensor_read_config *read_config =
		(struct sensor_read_config *)iodev->data;
	int err;

	LOG_INF("setting up stream %p...", iodev);

	/* Don't check error as it may only be configurable through DTS */
	(void)sensor_attr_set(read_config->sensor, SENSOR_CHAN_ALL,
				SENSOR_ATTR_BATCH_DURATION, &ticks_per_event);

	err = sensor_stream(iodev, rtio_ctx, (void *)iodev, NULL);
	if (err != 0) {
		LOG_ERR("Failed to start sensor-stream: %d, %p...", err, iodev);
		rtio_sqe_reset_all(ctx->stream.ctx);
		return err;
	}
	ctx->running = true;

	return 0;
}

static void imu_stream_thread(void *arg0)
{
	struct context *ctx = (struct context *)arg0;
	int err;

	LOG_INF("init");
	zros_node_init(&ctx->node, ctx->name);
	zros_pub_init(&ctx->pub_imu, &ctx->node, ctx->pub_topic, &ctx->imu);
	zros_sub_init(&ctx->sub_status, &ctx->node, &topic_status, &ctx->status, 1);
	filter_init(ctx);

	err = setup_stream(ctx);
	__ASSERT(!err, "Failed to start stream for %p", ctx->stream.iodev);

	while (true) {
		err = sensor_processing_cb_with_timeout(ctx->stream.ctx, process_events,
							K_MSEC(100));
		if (err != 0 || !ctx->running) {
			ctx->running = false;
			LOG_ERR_RATELIMIT_RATE(5000, "Error during stream. Attempting recovery...");
			drain_pending_cqes(ctx);
			filter_init(ctx);
			do {
				k_sleep(K_MSEC(100));
				(void)setup_stream(ctx);
			} while (!ctx->running);
		}
	}
}

#if IS_ENABLED(CONFIG_ZROS_SENSE_STREAM_IMU_DRDY_MODE)
#define IMU_STREAM_TRIGGER SENSOR_TRIG_DATA_READY
#else
#define IMU_STREAM_TRIGGER SENSOR_TRIG_FIFO_WATERMARK
#endif

#define IMU_STREAM_DEFINE(i)									   \
												   \
RTIO_DEFINE_WITH_MEMPOOL(imu_stream_ctx_##i, 2, 4, 128, 4, sizeof(void *));			   \
SENSOR_DT_STREAM_IODEV(imu_stream_iodev_##i, IMU_ALIAS(i),					   \
		       {IMU_STREAM_TRIGGER, SENSOR_STREAM_DATA_INCLUDE});		   \
												   \
static struct context imu_stream_context_##i = {						   \
	.name = STRINGIFY(sense_imu_##i),							   \
	.node = {},										   \
	.pub_imu = {},										   \
	.pub_topic = &topic_imu##i,								   \
	.imu = {										   \
		.has_stamp = true,								   \
		.stamp = synapse_pb_Timestamp_init_default,					   \
		.has_angular_velocity = true,							   \
		.angular_velocity = synapse_pb_Vector3_init_default,				   \
		.has_linear_acceleration = true,						   \
		.linear_acceleration = synapse_pb_Vector3_init_default,				   \
		.has_orientation = false,							   \
	},											   \
	.sub_status = {},									   \
	.status = synapse_pb_Status_init_default,						   \
	.running = false,									   \
	.calibration = {									   \
		.state = SENSE_IMU_STREAM_UNCALIBRATED,						   \
	},											   \
	.stream = {										   \
		.ctx = &imu_stream_ctx_##i,							   \
		.iodev = &imu_stream_iodev_##i,							   \
	},											   \
};												   \
												   \
K_THREAD_DEFINE(imu_stream_thread_##i##_id, 2048, imu_stream_thread,				   \
		&imu_stream_context_##i, NULL, NULL, 2, 0, 0)

#define IMU_STREAM_DEFINE_IF_EXISTS(i)								   \
	IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(i)), (IMU_STREAM_DEFINE(i)))

IMU_STREAM_DEFINE_IF_EXISTS(0);
IMU_STREAM_DEFINE_IF_EXISTS(1);
IMU_STREAM_DEFINE_IF_EXISTS(2);

#define IMU_STREAM_CTX_LIST									   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(0)), (&imu_stream_context_0,))		   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(1)), (&imu_stream_context_1,))		   \
		IF_ENABLED(DT_NODE_EXISTS(IMU_ALIAS(2)), (&imu_stream_context_2,))

struct context * const imu_list[] = {
		IMU_STREAM_CTX_LIST
};

static struct context *get_context_from_iodev(const struct rtio_iodev *iodev)
{
	for (size_t i = 0 ; i < ARRAY_SIZE(imu_list) ; i++) {
		if (imu_list[i] != NULL && imu_list[i]->stream.iodev == iodev) {
			return imu_list[i];
		}
	}
	CODE_UNREACHABLE;
}
