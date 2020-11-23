#ifndef DRM_BOOTSPLASH_H
#define DRM_BOOTSPLASH_H

#include <linux/types.h>

struct drm_device;

struct drm_bootsplash {
	struct drm_client_dev *client;
	struct drm_client_display *display;
	struct drm_client_buffer *buffer[2];
	struct work_struct worker;
	bool stop;
};

static inline struct drm_bootsplash* drm_bootsplash_from_client(struct drm_client_dev *client)
{
	return container_of(client, struct drm_bootsplash, client);
}

static u32 drm_bootsplash_color_table[3] = {
	0x00ff0000, 0x0000ff00, 0x000000ff,
};

int drm_bootsplash_init(struct drm_device *dev);

#endif