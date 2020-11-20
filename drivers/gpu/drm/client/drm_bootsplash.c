// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <drm/drmP.h>
#include <drm/drm_client.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>

struct drm_bootsplash {
	struct drm_client_dev *client;
	struct drm_client_display *display;
	struct drm_client_buffer *buffer[2];
	struct work_struct worker;
	bool stop;
};

static u32 drm_bootsplash_color_table[3] = {
	0x00ff0000, 0x0000ff00, 0x000000ff,
};

/* Draw a box with changing colors */
static void
drm_bootsplash_draw(struct drm_client_buffer *buffer, unsigned int sequence)
{
	unsigned int x, y;
	u32 *pix;

	pix = buffer->vaddr;
	pix += ((buffer->fb->height / 2) - 50) * buffer->fb->width;
	pix += (buffer->fb->width / 2) - 50;

	for (y = 0; y < 100; y++) {
		for (x = 0; x < 100; x++)
			*pix++ = drm_bootsplash_color_table[sequence];
		pix += buffer->fb->width - 100;
	}
}

static void drm_bootsplash_worker(struct work_struct *work)
{
	struct drm_bootsplash *splash = container_of(work, struct drm_bootsplash,
						     worker);
	struct drm_event *event;
	unsigned int i = 0, sequence = 0, fb_id;
	int ret;

	while (!splash->stop) {
		/* Are we still in charge? */
		// fb_id = drm_client_display_current_fb(splash->display);
		// if (fb_id != splash->buffer[i]->fb_ids[0])
		// 	break;

		/*
		 * We can race with userspace here between checking and doing
		 * the page flip, so double buffering isn't such a good idea.
		 * Tearing probably isn't a problem on a presumably small splash
		 * animation. I've kept it to test the page flip code.
		 */

		i = !i;
		drm_bootsplash_draw(splash->buffer[i], sequence++);
		if (sequence == 3)
			sequence = 0;

		ret = drm_client_display_page_flip(splash->display,
						   splash->buffer[i]->fb_ids[0],
						   true);
		if (!ret) {
			event = drm_client_read_event(splash->client, true);
			if (!IS_ERR(event))
				kfree(event);
		}
		msleep(500);
	}

	for (i = 0; i < 2; i++)
		drm_client_framebuffer_delete(splash->buffer[i]);
	drm_client_display_free(splash->display);
}

static int drm_bootsplash_setup(struct drm_bootsplash *splash)
{
	struct drm_client_dev *client = splash->client;
	struct drm_client_buffer *buffer[2];
	struct drm_client_display *display;
	struct drm_mode_modeinfo *mode;
	int ret, i;

	display = drm_client_display_get_first_enabled(client, false);
	if (IS_ERR(display))
		return PTR_ERR(display);
	if (!display)
		return -ENOENT;

	mode = drm_client_display_first_mode(display);
	if (!mode) {
		ret = -EINVAL;
		goto err_free_display;
	}

	for (i = 0; i < 2; i++) {
		buffer[i] = drm_client_framebuffer_create(client, mode,
							  DRM_FORMAT_XRGB8888);
		if (IS_ERR(buffer[i])) {
			ret = PTR_ERR(buffer[i]);
			goto err_free_buffer;
		}
	}

	ret = drm_client_display_commit_mode(display, buffer[0]->fb_ids[0], mode);
	if (ret)
		goto err_free_buffer;

	splash->display = display;
	splash->buffer[0] = buffer[0];
	splash->buffer[1] = buffer[1];

	schedule_work(&splash->worker);

	return 0;

err_free_buffer:
	for (i--; i >= 0; i--)
		drm_client_framebuffer_delete(buffer[i]);
err_free_display:
	drm_client_display_free(display);

	return ret;
}

static int drm_bootsplash_client_hotplug(struct drm_client_dev *client)
{
	struct drm_bootsplash *splash = client->private;
	int ret = 0;

	if (!splash->display)
		ret = drm_bootsplash_setup(splash);

	return ret;
}


static int drm_bootsplash_client_remove(struct drm_client_dev *client)
{
	struct drm_bootsplash *splash = client->private;

	if (splash->display) {
		splash->stop = true;
		flush_work(&splash->worker);
	}

	kfree(splash);

	return 0;
}

static const struct drm_client_funcs drm_bootsplash_client_funcs = {
	.owner		    = THIS_MODULE,
	.unregister		= drm_bootsplash_client_remove,
	.hotplug	    = drm_bootsplash_client_hotplug,
};

static int __init drm_bootsplash_init(void)
{
    int ret = 0;
    struct drm_bootsplash *splash;

	splash = kzalloc(sizeof(*splash), GFP_KERNEL);
	if (!splash)
		return -ENOMEM;

	INIT_WORK(&splash->worker, drm_bootsplash_worker);

	splash->client = client;
	client->private = splash;

	if (!splash->display) {
		ret = drm_bootsplash_setup(splash);

        if (ret) {
            printk("BOOTSPLASH: drm_bootsplash_setup failed: ret %d\n", ret);
            return ret;
        }
    }
    
	ret = drm_client_init(<DEV I NEED TO GET>, &client, "drm_bootsplash", &drm_bootsplash_client_funcs);
    if (ret) {
        printk("BOOTSPLASH: drm_client_init setup failed: ret %d\n", ret);
        return ret;
    }

    drm_client_register(&client);

    return ret;
}
module_init(drm_bootsplash_init);

static void __exit drm_bootsplash_exit(void)
{
	drm_client_unregister(&drm_bootsplash_client_funcs);
}
module_exit(drm_bootsplash_exit);

MODULE_LICENSE("GPL");