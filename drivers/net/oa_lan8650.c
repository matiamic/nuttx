static int oa_lan8650_init_mac(struct oa_driver_s *priv)
{
  /* do something */
}

static int oa_lan8650_config(struct oa_driver_s *priv)
{
  /* do something */
}

static int oa_lan8650_add_mac(struct oa_driver_s *priv, uint8_t *mac)
{
  /* do something */
}

static int oa_lan8650_rm_mac(struct oa_driver_s *priv, uint8_t *mac)
{
  /* do something */
}

static int oa_lan8650_ioctl(struct oa_driver_s *priv, int cmd,
                     unsigned long arg)
{
  /* do something */
}

struct oa_lan8650_driver_s
{
  struct oa_driver_s oa_dev;

  int somethingmore;
}

struct oa_ops_s g_oa_lan8650_ops =
{
  oa_lan8650_init_mac,
  oa_lan8650_config,
  oa_lan8650_add_mac,
  oa_lan8650_rm_mac,
  oa_lan8650_ioctl
};

struct oa_driver_s *oa_lan8650_initialize(struct spi_dev_s *spi,
                                          struct oa_config_s *config)
{
  FAR struct oa_lan8650_driver_s *priv = NULL;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("Could not allocate data for oa_lan8650_driver_s priv\n");
      return NULL;
    }

  /* Assign spi and config only if needed by the lan8650 init code, in any case it will be reassigned later in oa_initialize */

  priv->oa_dev.spi = spi;
  priv->oa_dev.config = config;

  /* Save the ops pointer */

  priv->oa_dev.ops = &g_oa_lan8650_ops;

  /* Do something with additional structure fields or with the device */

  /* Return */

  return priv->oa_dev;
}
